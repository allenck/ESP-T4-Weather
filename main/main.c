/*

MIT No Attribution

Copyright (c) 2020 Allen Kempe

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

-cut-

SPDX-License-Identifier: MIT-0

*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <wchar.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <esp_log.h>
#include <esp_task_wdt.h>

#include <unistd.h>

#include "sdkconfig.h"

#include <hagl_hal.h>
#include <hagl.h>
#include <bitmap.h>
#include <fps.h>
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_vfs_fat.h"
#include <sys/dirent.h>
#include <sys/stat.h>
#include <ctype.h>
#include <esp_err.h>
#include <mipi_display.h>
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include <cJSON.h>
#include "esp_eth.h"
#include "esp_http_client.h"
#include "font_render.h"
#include "hagl_util.h"
#include "unicode.h"
#include "lwip/apps/sntp.h"
#include "font6x9.h"
#include "esp_heap_caps_init.h"
#include "lwip/err.h"
#include "lwip/sys.h"

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static const char *TAG = "main";

static bitmap_t *bb;
static EventGroupHandle_t event;
static EventGroupHandle_t s_wifi_event_group;

//static const uint8_t FRAME_LOADED = (1 << 0);

// hard coded for now
#define MIPI_DISPLAY_PIN_BUTTON_A 37
#define MIPI_DISPLAY_PIN_BUTTON_B 38
#define MIPI_DISPLAY_PIN_BUTTON_C 39
#define GPIO_INPUT_IO_0     MIPI_DISPLAY_PIN_BUTTON_A
#define GPIO_INPUT_IO_1     MIPI_DISPLAY_PIN_BUTTON_B
#define GPIO_INPUT_IO_2     MIPI_DISPLAY_PIN_BUTTON_C
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1)| (1ULL<<GPIO_INPUT_IO_2))
#define ESP_INTR_FLAG_DEFAULT 0
#define BUFFER_SIZE DISPLAY_HEIGHT

static xQueueHandle gpio_evt_queue = NULL;
char* jsonBuffer = NULL;
size_t jsonBuffer_len = 0;

cJSON* current = NULL;
cJSON *root = NULL;
cJSON* weather_array = NULL;
cJSON* weather =NULL;
cJSON* hourly = NULL;
cJSON* hourly_array = NULL;
cJSON* daily = NULL;
cJSON* daily_array = NULL;

extern const uint8_t ttf_start[] asm("_binary_Ubuntu_R_ttf_start");
extern const uint8_t ttf_end[] asm("_binary_Ubuntu_R_ttf_end");
static hagl_driver_t driver;

static font_render_t font_render_10;
static font_render_t font_render_14;
static font_render_t font_render_16;
static font_render_t font_render_24;
static font_face_t font_face;

#define DRAW_EVENT_START 0xfffc
#define DRAW_EVENT_END 0xfffd
#define DRAW_EVENT_FRAME_START 0xfffe
#define DRAW_EVENT_FRAME_END 0xffff
#define DRAW_EVENT_CONTROL DRAW_EVENT_START
#define BUFFER_SIZE DISPLAY_HEIGHT
static color_t WHITE;
static color_t BLACK;
static color_t BLUE;
static color_t LTGRAY;
static color_t ORANGE;
static color_t YELLOW;
static color_t GREEN;
static color_t RED;

static void Draw_Heading_Section();
static void Draw_Main_Weather_Section();
static void DisplayWXicon(int x, int y, char* IconName, bool LargeSize);
static void DrawWind(int x, int y, float angle, float windspeed);
static void arrow(int x, int y, int asize, float aangle, int pwidth, int plength);
static char* WindDegToDirection(float winddirection);
static void DrawPressureTrend(int x, int y, float pressure, char* slope);
static void Draw_3hr_Forecast(int x, int y, int index);
static void Draw_Astronomy_Section();
static time_t convertTime(long dt);
static int currentHour();
static int getHour(time_t time);
static void format_time(char* dest, time_t time);

#define PI 3.1415
typedef struct draw_event_param {
    uint64_t frame;
    uint64_t total_frame;
    uint64_t duration;
    void *user_data;
} draw_event_param_t;

char  *time_str, *Day_time_str; // strings to hold time and received weather data;
char *ipAddr;
char * Units = "M";
int s_retry_num = 0;
bool connected = false;
bool    Largesize  = true;
bool    Smallsize  = false;
#define Large 7
#define Small 3
char * currWeatherIcon;
char * currentTemperature;
float windSpeed;
int windDir;
float currentRain =0;
float currentPressure;
char* pressureTrend="0";
char* currentDescription;
long tzOffset=0;

static bool sntp_time_started = false;
static bool time_display_started = false;
void time_cb(struct timeval *tv);

esp_err_t _http_event_handle(esp_http_client_event_t *evt);

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task(void* arg)
{
    uint32_t io_num;

    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
//            printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
            if(io_num == MIPI_DISPLAY_PIN_BUTTON_A )
            {
            }
        }

    }
}

static void setup_Gpio()
{
    gpio_config_t io_conf;
    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    //change gpio intrrupt type for one pin
    //gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_ANYEDGE);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(gpio_task, "gpio_task_example", 2048, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);
    gpio_isr_handler_add(GPIO_INPUT_IO_2, gpio_isr_handler, (void*) GPIO_INPUT_IO_2);
}

esp_err_t _http_event_handle(esp_http_client_event_t *evt)
{
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGI(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGI(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_HEADER");
            printf("%.*s", evt->data_len, (char*)evt->data);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            if (!esp_http_client_is_chunked_response(evt->client)) {
                printf("%.*s", evt->data_len, (char*)evt->data);
                if(jsonBuffer == 0)
                {
                    jsonBuffer = (char*)malloc(evt->data_len);
                    memcpy(jsonBuffer, (char*)evt->data, evt->data_len);
                    jsonBuffer_len = evt->data_len;
                }
                else
                {
                    jsonBuffer = (char*)realloc(jsonBuffer, jsonBuffer_len +evt->data_len);
                    memcpy(jsonBuffer + jsonBuffer_len, (char*)evt->data, evt->data_len);
                    jsonBuffer_len += evt->data_len;
                }

            }
            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGI(TAG, "HTTP_EVENT_ON_FINISH");
            if(root)
            {
                free(root);
            }
            root = cJSON_Parse(jsonBuffer);
            tzOffset = cJSON_GetObjectItem(root, "timezone_offset")->valueint;
            current = cJSON_GetObjectItem(root, "current");
            hourly = cJSON_GetObjectItem(root, "hourly");
            daily = cJSON_GetObjectItem(root, "daily");
#if 1
            if(current)
            {
                ESP_LOGI(TAG, "current found!");
                cJSON* dt = cJSON_GetObjectItem(current, "dt");
                cJSON* temp = cJSON_GetObjectItem(current, "temp");
                cJSON* humidity = cJSON_GetObjectItem(current, "humidity");
                cJSON* pressure = cJSON_GetObjectItem(current, "pressure");
                cJSON* dewPoint = cJSON_GetObjectItem(current, "dew_point");
                cJSON* wind_speed = cJSON_GetObjectItem(current, "wind_speed");
                windSpeed = wind_speed->valuedouble;
                ESP_LOGI(TAG, "currentWindSpeed=%f", windSpeed);
                cJSON* wind_deg = cJSON_GetObjectItem(current, "wind_deg");
                windDir = wind_deg->valueint;
                weather_array = cJSON_GetObjectItem(current, "weather");
                cJSON* rain = cJSON_GetObjectItem(current, "rain");
                char *string = NULL;
//                char16_t message[40];
                if(temp)
                {
                    ESP_LOGI(TAG, "temp found!");
                    time_t utc;
                    char* pEnd;
                    char strftime_buf[64];
                    struct tm timeinfo;
                    string = cJSON_Print(dt);
                    utc = (time_t)strtol(string, &pEnd,10);
                    localtime_r(&utc, &timeinfo);
                    strftime(strftime_buf, sizeof(strftime_buf), "%T", &timeinfo);
                    ESP_LOGI(TAG, "The last report at: %s", strftime_buf);
//                    swprintf(message, sizeof(message), u"%s    ", strftime_buf);
//                    hagl_put_text(message, 4, 41, hagl_color(255, 0, 255), fnt9x18b);

                    string = cJSON_Print(temp);
                    if(string)
                    {
                        currentTemperature = strdup(unQuote(string));
                        ESP_LOGI(TAG, "temperature = %s", string);
                        char* pEnd;
                        double tC = strtod(string, &pEnd);
                        double tF = (tC*9/5) +32;
                        int rh = strtol(cJSON_Print(humidity), &pEnd, 10);

//                        swprintf(message, sizeof(message), L"%.*f°C %.*f°F rh: %d%%", 0, tC,0, tF, rh);
//                        hagl_put_text(message, 4, 61, hagl_color(255, 255, 0), fnt9x18b);
                        free(string);

                        string = cJSON_Print(pressure);
                        currentPressure = pressure->valuedouble;
                        double pM = strtod(string, &pEnd);
                        double pI = pM * 0.02953;
//                        swprintf(message, sizeof(message), L"bar: %.*f mb / %.*f\"", 0, pM,0, pI);
//                        hagl_put_text(message, 4, 101, hagl_color(255, 255, 0), fnt9x18b);
                        free(string);

                        string = cJSON_Print(dewPoint);
                        tC = strtod(string, &pEnd);
                        tF = (tC*9/5) +32;
//                        swprintf(message, sizeof(message), L"dew point: %.*f°C %.*f°F", 0, tC,0, tF);
//                        hagl_put_text(message, 4, 121, hagl_color(0, 255, 0), fnt9x18b);
                        free(string);

                        if(rain)
                        {
                            currentRain = rain->valuedouble;
                            cJSON* lastHr = cJSON_GetObjectItem(rain, "1h");
                            if(lastHr)
                            {
                                char *string = cJSON_Print(lastHr);
                                ESP_LOGI(TAG, "rain %s", string);
                                double dRain = strtod(string, &pEnd);
//                                swprintf(message, sizeof(message), L"rain: %.2f\"", 0, dRain);
//                                hagl_put_text(message, 4, 141, hagl_color(0, 0, 255), fnt9x18b);
                                free(string);
                                free(lastHr);
                            }
                            else{
                                ESP_LOGI(TAG, "no rain lh");
                            }
                            free(rain);
                        }
                    else{
                                ESP_LOGI(TAG, "no rain object");
                        }
                    }
                    free(dt);
                    free(temp);
                    free(humidity);
                    free(pressure);
                    free(dewPoint);
                }
                if(weather_array && cJSON_IsArray(weather_array))
                {
                    ESP_LOGI(TAG, "weather array found.");
                    ESP_LOGI(TAG, "weather %d items", cJSON_GetArraySize(weather_array));
                    weather = cJSON_GetArrayItem(weather_array, 0);
                    if(weather)
                    {
                        cJSON* description = cJSON_GetObjectItem(weather, "description");
                        currentDescription = description->valuestring;
                        cJSON* icon = cJSON_GetObjectItem(weather, "icon");
                        char* string = cJSON_Print(description);
//                        char16_t message[20];
//                        swprintf(message, sizeof(message), u"%s    ", string);
//                        hagl_put_text(message, 0, 81, hagl_color(255, 0, 0), fnt9x18b );
                        currWeatherIcon = strdup(unQuote(cJSON_Print(icon)));
                        ESP_LOGI(TAG, "currentWeatherIcon=%s", currWeatherIcon);
                        free(description);
                        free(string);
                        free(weather);
                        free(icon);
                    }
                    free(weather_array);
                }
                else
                {
                    ESP_LOGI(TAG, "weather array not found");
                }
                free(current);
            }
            if(hourly)
            {

                ESP_LOGI(TAG, "hourly items = %d", cJSON_GetArraySize(hourly));
                //free(hourly);
            }
            else
            {
              ESP_LOGE(TAG, "hourly not found");
            }
            if(daily)
            {

                ESP_LOGI(TAG, "daily items = %d", cJSON_GetArraySize(daily));
                //free(hourly);
            }
            else
            {
              ESP_LOGE(TAG, "daily not found");
            }
#endif
            WHITE  = hagl_color(255,255,255);
            BLACK  = hagl_color(0,0,0);
            BLUE   = hagl_color(0,0,255);
            LTGRAY = hagl_color(200,200,200);
            ORANGE = hagl_color(255,127,0);
            YELLOW = hagl_color(244,196,48);
            GREEN  = hagl_color(0,255,0);
            RED    = hagl_color(255,0,0);

            Draw_Heading_Section();
            Draw_Main_Weather_Section();     // Centre section of display for Location, temperature, Weather report, cWx Symbol and wind direction
            Draw_3hr_Forecast(20,102, 1);    // First  3hr forecast box
            Draw_3hr_Forecast(70,102, 2);    // Second 3hr forecast box
            Draw_3hr_Forecast(120,102,3);    // Third  3hr forecast box
            Draw_Astronomy_Section();        // Astronomy section Sun rise/set, Moon phase and Moon icon
            free(root);
            root = NULL;
            if(hourly)
            {
                free(hourly);
                hourly = NULL;
            }
            if(hourly_array)
            {
                free(hourly_array);
                hourly_array = NULL;
            }
            if(daily)
            {
                free(daily);
                daily = NULL;
            }
            if(daily_array)
            {
                free(daily_array);
                daily_array = NULL;
            }
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED");
            break;
    }
    return ESP_OK;
}
#ifdef HAGL_HAL_USE_BUFFERING

/*
 * Flush backbuffer to display always when new frame is loaded.
 */
void flush_task(void *params)
{
    ESP_LOGI(TAG, "flush task started");
    while (1) {
        EventBits_t bits = xEventGroupWaitBits(
            event,
            FRAME_LOADED,
            pdTRUE,
            pdFALSE,
            40 / portTICK_PERIOD_MS
        );

        /* Flush only when FRAME_LOADED is set. */
        if ((bits & FRAME_LOADED) != 0 ) {
            hagl_flush();
        }

    }

    vTaskDelete(NULL);
}
#endif
/*
 * Software vsync. Waits for flush to start. Needed to avoid
 * tearing when using double buffering, NOP otherwise. This
 * could be handler with IRQ's if the display supports it.
 */
static void wait_for_vsync()
{
#ifdef CONFIG_HAGL_HAL_USE_DOUBLE_BUFFERING
    xEventGroupWaitBits(
        event,
        FRAME_LOADED,
        pdTRUE,
        pdFALSE,
        10000 / portTICK_RATE_MS
    );
    ets_delay_us(25000);
#endif /* CONFIG_HAGL_HAL_USE_DOUBLE_BUFFERING */
}

void start_NTP()
{
    // start sntp NTP client
    ESP_LOGI(TAG,"start sntp NTP client");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_set_time_sync_notification_cb(time_cb);
    sntp_init();
    sntp_time_started = true;
}

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        ESP_LOGI(TAG, "got ip:%s", ipAddr =
                 ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        connected = true;

        //hagl_fill_rectangle(70,50,170, 150, 0); // erase completely

        if(!time_display_started)
        {
            start_NTP();
            time_display_started = true;
        }

        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
    {
        if (s_retry_num < CONFIG_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
            s_retry_num++;
            ESP_LOGI(TAG,"retry to connect to the AP");
        }
        ESP_LOGI(TAG,"connect to the AP fail\n");
        connected = false;
        break;
    }
    default:
        break;
    }
    return ESP_OK;
}
/*
 * Runs the actual demo effect.
 */
void main_task(void *params)
{
    int url_len;
    char u1[] = "https://api.openweathermap.org/data/2.5/onecall?lat=";
    char u2[] = CONFIG_WEATHER_LATITUDE;
    char u3[] = "&lon=";
    char u4[] = CONFIG_WEATHER_LONGITUDE;
    char u5[] = "&units=metric&lang=en&exclude=minutely&appid=";
    char u6[] = CONFIG_WEATHER_API_KEY;
    char *url = (char *)malloc(strlen(u1)+strlen(u2)+strlen(u3)+strlen(u4)+strlen(u5)+strlen(u6)+1);
    strcpy(url, u1);
    url_len = strlen(u1);
    strcpy(url+url_len, u2);
    url_len+=strlen(u2);
    strcpy(url+url_len, u3);
    url_len+=strlen(u3);
    strcpy(url+url_len, u4);
    url_len+=strlen(u4);
    strcpy(url+url_len, u5);
    url_len+=strlen(u5);
    strcpy(url+url_len, u6);
    url_len+=strlen(u6);
    esp_http_client_config_t config = {
    .url = "https://api.openweathermap.org/data/2.5/onecall?lat=37.98&lon=-85.71&units=metric&lang=en&exclude=minutely,hourly&appid=????????",
    .event_handler = _http_event_handle,
    };
    config.url = url;
    ESP_LOGI(TAG, "url = '%s'", url);
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t err = esp_http_client_perform(client);

    if (err == ESP_OK) {
    ESP_LOGI(TAG, "Status = %d, content_length = %d",
            esp_http_client_get_status_code(client),
            esp_http_client_get_content_length(client));
    }
    esp_http_client_cleanup(client);

    vTaskDelete(NULL);
}

static int render_text(const char *text, font_render_t *render, hagl_driver_t *driver, int src_x, int src_y, int y, uint8_t color_r, uint8_t color_g, uint8_t color_b) {
    if (src_y - y >= BUFFER_SIZE || src_y + (int)render->max_pixel_height - y < 0) {
        return 0;
    }
    ESP_LOGI(TAG, "text=%s, x=%d, y=%d", text, src_x, src_y);
    while (*text) {
        uint32_t glyph;
        const char c = *text;
        text += u8_decode(&glyph, text);
        font_render_glyph(render, glyph);
        hagl_draw_gray2_bitmap(render->bitmap, driver->current_buffer, color_r, color_g, color_b, src_x + render->bitmap_left,
                               render->max_pixel_height - render->origin - render->bitmap_top + src_y - y,
                               render->bitmap_width, render->bitmap_height, driver->display_width, /*BUFFER_SIZE*/driver->display_height);
        src_x += render->advance;
    }
    return src_x;
}

//#########################################################################################
void UpdateLocalTime(){
  time_t now;
  struct tm timeinfo;
  time(&now);
  localtime_r(&now, &timeinfo);
  //See http://www.cplusplus.com/reference/ctime/strftime/
  //Serial.println(&timeinfo, "%a %b %d %Y   %H:%M:%S"); // Displays: Saturday, June 24 2017 14:05:49
  //Serial.println(&timeinfo, "%H:%M:%S"); // Displays: 14:05:49
  char output[30], day_output[34];
  if (strcmp(Units, "M")==0) {
    strftime(day_output, 34, "%a  %d-%B-%Y", &timeinfo); // Creates: Sat 24-Jun-17
    strftime(output, 30, "(@ %H:%M:%S )", &timeinfo);    // Creates: 14:05:49
  }
  else {
    strftime(day_output, 34, "%a  %B-%d-%Y", &timeinfo); // Creates: Sat Jun-24-17
    strftime(output, 30, "(@ %r )", &timeinfo);          // Creates: 2:05:49pm
  }
  //Day_time_str = day_output;
  if(Day_time_str)
      free(Day_time_str);
  Day_time_str = strdup(day_output);
  //time_str     = output;
  if(time_str)
      free(time_str);
  time_str = strdup(output);
}
//#########################################################################################
// Symbols are drawn on a relative 10x10grid and 1 scale unit = 1 drawing unit
void addcloud(int x, int y, int scale, int linesize) {
    ESP_LOGI("addCloud", " x=%d, y=%d, %d %d", x, y, scale, linesize);

  //Draw cloud outer
  hagl_fill_circle(x-scale*3, y, scale, BLACK);                           // Left most circle
  hagl_fill_circle(x+scale*3, y, scale, BLACK);                           // Right most circle
  hagl_fill_circle(x-scale, y-scale, scale*1.4, BLACK);                   // left middle upper circle
  hagl_fill_circle(x+scale*1.5, y-scale*1.3, scale*1.75, BLACK);          // Right middle upper circle
  hagl_fill_rect_wh(x-scale*3-1, y-scale, scale*6, scale*2+1, BLACK);   // Upper and lower lines
  //Clear cloud inner
  //gfx.setColor(EPD_WHITE);
  hagl_fill_circle(x-scale*3, y, scale-linesize, LTGRAY);                  // Clear left most circle
  hagl_fill_circle(x+scale*3, y, scale-linesize, LTGRAY);                  // Clear right most circle
  hagl_fill_circle(x-scale, y-scale, scale*1.4-linesize, LTGRAY);          // left middle upper circle
  hagl_fill_circle(x+scale*1.5, y-scale*1.3, scale*1.75-linesize, LTGRAY); // Right middle upper circle
  hagl_fill_rect_wh(x-scale*3+2, y-scale+linesize-1, scale*5.9, scale*2-linesize*2+2, LTGRAY); // Upper and lower lines
  //gfx.setColor(EPD_BLACK);
}
//#########################################################################################
void addrain(int x, int y, int scale){
    ESP_LOGI("addRain", "scale=%d", scale);
  y = y + scale/2;
  for (int i = 0; i < 6; i++){
    hagl_draw_line(x-scale*4+scale*i*1.3+0, y+scale*1.9, x-scale*3.5+scale*i*1.3+0, y+scale, BLUE);
    if (scale != Small) {
      hagl_draw_line(x-scale*4+scale*i*1.3+1, y+scale*1.9, x-scale*3.5+scale*i*1.3+1, y+scale, BLUE);
      hagl_draw_line(x-scale*4+scale*i*1.3+2, y+scale*1.9, x-scale*3.5+scale*i*1.3+2, y+scale, BLUE);
    }
  }
}
//#########################################################################################
void addsnow(int x, int y, int scale){
    ESP_LOGI("addSnow", "scale=%d", scale);
  int dxo, dyo, dxi, dyi;
  for (int flakes = 0; flakes < 5;flakes++){
    for (int i = 0; i <360; i = i + 45) {
      dxo = 0.5*scale * cos((i-90)*3.14/180); dxi = dxo*0.1;
      dyo = 0.5*scale * sin((i-90)*3.14/180); dyi = dyo*0.1;
      hagl_draw_line(dxo+x+0+flakes*1.5*scale-scale*3,dyo+y+scale*2,dxi+x+0+flakes*1.5*scale-scale*3,dyi+y+scale*2, BLACK);
    }
  }
}
//#########################################################################################
void addtstorm(int x, int y, int scale){
    ESP_LOGI("addTstorm", "scale=%d", scale);
  y = y + scale/2;
  for (int i = 0; i < 5; i++){
    hagl_draw_line(x-scale*4+scale*i*1.5+0, y+scale*1.5, x-scale*3.5+scale*i*1.5+0, y+scale, BLACK);
    if (scale != Small) {
      hagl_draw_line(x-scale*4+scale*i*1.5+1, y+scale*1.5, x-scale*3.5+scale*i*1.5+1, y+scale, BLACK);
      hagl_draw_line(x-scale*4+scale*i*1.5+2, y+scale*1.5, x-scale*3.5+scale*i*1.5+2, y+scale, BLACK);
    }
    hagl_draw_line(x-scale*4+scale*i*1.5, y+scale*1.5+0, x-scale*3+scale*i*1.5+0, y+scale*1.5+0, BLACK);
    if (scale != Small) {
      hagl_draw_line(x-scale*4+scale*i*1.5, y+scale*1.5+1, x-scale*3+scale*i*1.5+0, y+scale*1.5+1, BLACK);
      hagl_draw_line(x-scale*4+scale*i*1.5, y+scale*1.5+2, x-scale*3+scale*i*1.5+0, y+scale*1.5+2, BLACK);
    }
    hagl_draw_line(x-scale*3.5+scale*i*1.4+0, y+scale*2.5, x-scale*3+scale*i*1.5+0, y+scale*1.5, BLACK);
    if (scale != Small) {
      hagl_draw_line(x-scale*3.5+scale*i*1.4+1, y+scale*2.5, x-scale*3+scale*i*1.5+1, y+scale*1.5, BLACK);
      hagl_draw_line(x-scale*3.5+scale*i*1.4+2, y+scale*2.5, x-scale*3+scale*i*1.5+2, y+scale*1.5, BLACK);
    }
  }
}
//#########################################################################################
void addsun(int x, int y, int scale) {
    ESP_LOGI("addSun", "scale=%d", scale);
  int linesize = 3;
  if (scale == Small) linesize = 1;
  int dxo, dyo, dxi, dyi;
  hagl_fill_circle(x, y, scale, YELLOW);
  //gfx.setColor(EPD_WHITE);
  hagl_fill_circle(x, y, scale-linesize, ORANGE);
  //gfx.setColor(EPD_BLACK);
  for (float i = 0; i <360; i = i + 45) {
    dxo = 2.2*scale * cos((i-90)*3.14/180); dxi = dxo * 0.6;
    dyo = 2.2*scale * sin((i-90)*3.14/180); dyi = dyo * 0.6;
    if (i == 0   || i == 180) {
      hagl_draw_line(dxo+x-1,dyo+y,dxi+x-1,dyi+y, YELLOW);
      if (scale != Small) {
        hagl_draw_line(dxo+x+0,dyo+y,dxi+x+0,dyi+y, YELLOW);
        hagl_draw_line(dxo+x+1,dyo+y,dxi+x+1,dyi+y, YELLOW);
      }
    }
    if (i == 90  || i == 270) {
      hagl_draw_line(dxo+x,dyo+y-1,dxi+x,dyi+y-1, YELLOW);
      if (scale != Small) {
        hagl_draw_line(dxo+x,dyo+y+0,dxi+x,dyi+y+0, YELLOW);
        hagl_draw_line(dxo+x,dyo+y+1,dxi+x,dyi+y+1, YELLOW);
      }
    }
    if (i == 45  || i == 135 || i == 225 || i == 315) {
      hagl_draw_line(dxo+x-1,dyo+y,dxi+x-1,dyi+y, YELLOW);
      if (scale != Small) {
        hagl_draw_line(dxo+x+0,dyo+y,dxi+x+0,dyi+y, YELLOW);
        hagl_draw_line(dxo+x+1,dyo+y,dxi+x+1,dyi+y, YELLOW);
      }
    }
  }
}
//#########################################################################################
void addfog(int x, int y, int scale){
    ESP_LOGI("addFog", "scale=%d", scale);
  int linesize = 2;
  if (scale == Small) linesize = 1;
  for (int i = 0; i < 6; i++){
    hagl_fill_rect_wh(x-scale*3, y+scale*1.5, scale*6, linesize, LTGRAY);
    hagl_fill_rect_wh(x-scale*3, y+scale*2.0, scale*6, linesize, LTGRAY);
    hagl_fill_rect_wh(x-scale*3, y+scale*2.7, scale*6, linesize, LTGRAY);
  }
}
//#########################################################################################
void MostlyCloudy(int x, int y, int scale){
    ESP_LOGI("MostlyCloudy", "scale=%d", scale);
  int linesize = 3;
  if (scale == Small) linesize = 1;
  addcloud(x,y,scale,linesize);
  addsun(x-scale*1.8,y-scale*1.8,scale);
  addcloud(x,y,scale,linesize);
}
//#########################################################################################
void MostlySunny(int x, int y, int scale){
    ESP_LOGI("MostlySunny", "scale=%d", scale);
  int linesize = 3;
  if (scale == Small) linesize = 1;
  addcloud(x,y,scale,linesize);
  addsun(x-scale*1.8,y-scale*1.8,scale);
}
//#########################################################################################
void Rain(int x, int y, int scale){
    ESP_LOGI("Rain", "scale=%d", scale);
  int linesize = 3;
  if (scale == Small) linesize = 1;
  addcloud(x,y,scale,linesize);
  addrain(x,y,scale);
}
//#########################################################################################
void ProbRain(int x, int y, int scale){
    ESP_LOGI("Prob rain", "scale=%d", scale);
  x = x + 20;
  y = y + 15;
  addcloud(x,y,scale,1);
  y = y + scale/2;
  for (int i = 0; i < 6; i++){
    hagl_draw_line(x-scale*4+scale*i*1.3+0, y+scale*1.9, x-scale*3.5+scale*i*1.3+0, y+scale, BLUE);
  }
}
//#########################################################################################
void Cloudy(int x, int y, int scale){
    ESP_LOGI("Cloudy", "scale=%d", scale);
  int linesize = 3;
  if (scale == Small) linesize = 1;
  addcloud(x,y,scale,linesize);
}
//#########################################################################################
void Sunny(int x, int y, int scale){
    ESP_LOGI("Sunny", "scale=%d", scale);
  scale = scale * 1.45;
  addsun(x,y-4,scale);
}
//#########################################################################################
void ExpectRain(int x, int y, int scale){
    ESP_LOGI("ExpectRain", "scale=%d", scale);
  int linesize = 3;
  if (scale == Small) linesize = 1;
  addsun(x-scale*1.8,y-scale*1.8,scale);
  addcloud(x,y,scale,linesize);
  addrain(x,y,scale);
}
//#########################################################################################
void ChanceRain(int x, int y, int scale){
    ESP_LOGI("ChanceRain", "scale=%d", scale);
  int linesize = 3;
  if (scale == Small) linesize = 1;
  addsun(x-scale*1.8,y-scale*1.8,scale);
  addcloud(x,y,scale,linesize);
  addrain(x,y,scale);
}
//#########################################################################################
void Tstorms(int x, int y, int scale){
    ESP_LOGI("Tstorms", "scale=%d", scale);
  int linesize = 3;
  if (scale == Small) linesize = 1;
  addcloud(x,y,scale,linesize);
  addtstorm(x,y,scale);
}
//#########################################################################################
void Snow(int x, int y, int scale){
    ESP_LOGI("Snow", "scale=%d", scale);
  int linesize = 3;
  if (scale == Small) linesize = 1;
  addcloud(x,y,scale,linesize);
  addsnow(x,y,scale);
}
//#########################################################################################
void Fog(int x, int y, int scale){
    ESP_LOGI("Fog", "scale=%d", scale);
  int linesize = 3;
  if (scale == Small) linesize = 1;
  addcloud(x,y,scale,linesize);
  addfog(x,y,scale);
}
//#########################################################################################
void Haze(int x, int y, int scale){
    ESP_LOGI("Haze", "scale=%d", scale);
  int linesize = 3;
  if (scale == Small) linesize = 1;
  addsun(x,y,scale);
  addfog(x,y,scale);
}
//#########################################################################################
void Nodata(int x, int y, int scale){
  if (scale > Small) {
    //gfx.setFont(ArialMT_Plain_24);
    //gfx.drawString(x-10,y-18,"N/A");
      render_text("N/A", &font_render_24, &driver, x-10,y-18, 0, 0,0,0);
  }
  else
  {
    //gfx.setFont(ArialMT_Plain_10);
    //gfx.drawString(x-8,y-8,"N/A");
      render_text("N/A", &font_render_10, &driver, x-8,y-8, 0, 0,0,0);
  }
  //gfx.setFont(ArialMT_Plain_10);
}

void Draw_Heading_Section(){
    ESP_LOGI(TAG, "begin Draw_Heading_Section");
    hagl_clear_screen_to_color(WHITE);
    driver.current_buffer = &WHITE;
    driver.display_width = DISPLAY_WIDTH;
    driver.display_height = DISPLAY_HEIGHT;
    driver.buffer_size = DISPLAY_WIDTH * (DISPLAY_DEPTH / 8) * DISPLAY_HEIGHT;
    driver.bitmap = NULL;
    ESP_ERROR_CHECK(font_render_init(&font_render_10, &font_face, 10, 48));
    ESP_ERROR_CHECK(font_render_init(&font_render_16, &font_face, 16, 48));
    ESP_ERROR_CHECK(font_render_init(&font_render_24, &font_face, 24, 48));
    ESP_ERROR_CHECK(font_render_init(&font_render_14, &font_face, 14, 48));


//  gfx.setFont(ArialMT_Plain_16);
//  gfx.drawString(5,15,String(City));
    render_text(CONFIG_WEATHER_CITY, &font_render_16, &driver, 5, 15, 0, 0, 0, 255);
    //hagl_blit(0,0,&bm);
//  gfx.setFont(ArialMT_Plain_10);
//  gfx.drawString(5,0,time_str);
    render_text(time_str, &font_render_10, &driver, 5, 0, 0, 0,0,0);
//  gfx.drawString(185,0,Day_time_str);
    render_text(Day_time_str, &font_render_10, &driver, 190, 0, 0, 0,0,0);
//  hagl_draw_line(0,14,296,14);
    hagl_draw_hline(0, 14, DISPLAY_WIDTH, hagl_color(0,0,0));
}

void Draw_Main_Weather_Section(){
    ESP_LOGI(TAG, "begin Draw_Main_Weather_Section");
  //Period-0 (Main Icon/Report)
  DisplayWXicon(205,45,currWeatherIcon,Largesize);
//  gfx.setFont(ArialMT_Plain_24);
//  gfx.drawString(5,32,String(WxConditions[0].Temperature,1)+"°");
  int tl = render_text(currentTemperature, &font_render_24, &driver, 5, 32, 0, 0,0,0);
//  gfx.setFont(ArialRoundedMTBold_14);
//  gfx.drawString(String(WxConditions[0].Temperature,1).length()*13+8,33,(Units=="I"?"°F":"°C"));
  render_text(Units=='I'?"°F":"°C", &font_render_14, &driver, tl, 33, 0, 0,0,0);
//  gfx.setFont(ArialMT_Plain_10);
  DrawWind(265,42,windDir,windSpeed);
//  if (WxConditions[0].Rainfall > 0) gfx.drawString(90,35,String(WxConditions[0].Rainfall,1)+(Units=="M"?"mm":"in")+" of rain");
  if(currentRain)
  {
      char text[20];
      sprintf(text, "%.2f%s of rain", currentRain, (Units=='M'?"mm":"in"));
      render_text(text, &font_render_10, &driver, 90,35, 0, 0,0,0);
  }
  DrawPressureTrend(130,52,currentPressure,pressureTrend);
//  gfx.setFont(ArialRoundedMTBold_14);
//  String Wx_Description = WxConditions[0].Forecast0;
  render_text(currentDescription, &font_render_14, &driver, 130,52, 0, 0,0,0);
//  if (WxConditions[0].Forecast1 != "") Wx_Description += " & " +  WxConditions[0].Forecast1;
//    if (WxConditions[0].Forecast2 != "" && WxConditions[0].Forecast1 != WxConditions[0].Forecast2) Wx_Description += " & " +  WxConditions[0].Forecast2;
//  gfx.drawString(5,59, Wx_Description);
//  gfx.setFont(ArialMT_Plain_10);
  hagl_draw_line(0,77,DISPLAY_WIDTH,77, BLACK);
}
//#########################################################################################
void Draw_3hr_Forecast(int x, int y, int index){
  cJSON* hour = cJSON_GetArrayItem(hourly, currentHour() + index*3);
  int dt = cJSON_GetObjectItem(hour, "dt")->valueint + tzOffset;
  convertTime(dt);
  cJSON* weather_array = cJSON_GetObjectItem(hour, "weather");
  cJSON* weather = cJSON_GetArrayItem(weather_array, 0);
  DisplayWXicon(x+2,y,cJSON_GetObjectItem(weather,"icon")->valuestring,Smallsize);
  //gfx.setTextAlignment(TEXT_ALIGN_CENTER);
//  gfx.drawString(x+3,y-25,WxForecast[index].Period.substring(11,16));
  char* text[8];
  sprintf(text, "%d:00", getHour(convertTime(dt)));
  render_text(text, &font_render_14, &driver, x/*+3*/-3,y-25, 0, 0,0,0);
//  gfx.drawString(x+2,y+11,String(WxForecast[index].High,0)+"° / "+String(WxForecast[index].Low,0)+"°");
  hagl_draw_line(x+28,77,x+28,129, BLACK);
  //gfx.setTextAlignment(TEXT_ALIGN_LEFT);
}
//#########################################################################################
void Draw_Astronomy_Section(){
//  gfx.drawString(152,76,"Sun Rise: " + ConvertUnixTime(WxConditions[0].Sunrise).substring(0,5));
    cJSON* today = cJSON_GetArrayItem(daily, 0);
    long sunrise = cJSON_GetObjectItem(today, "sunrise")->valueint;
    long sunset = cJSON_GetObjectItem(today, "sunset")->valueint;
    char text[20];
    char tstr[10];
    format_time(tstr, sunrise + tzOffset);
    sprintf(text, "Sun Rise: %s", tstr);
    render_text(text, &font_render_14, &driver, 152,76, 0, 0,0,0);
//  gfx.drawString(178,88,"Set: "      + ConvertUnixTime(WxConditions[0].Sunset).substring(0,5));
    format_time(tstr, sunset + tzOffset);
    sprintf(text, "  Set: %s", tstr);
    render_text(text, &font_render_14, &driver, 178,88, 0, 0,0,0);
//  DrawMoon(230,65,MoonDay,MoonMonth,MoonYear,Hemisphere);

//  gfx.drawString(152,100,"Moon phase:");
//  gfx.drawString(152,112,MoonPhase(MoonDay,MoonMonth,MoonYear));
}
//#########################################################################################
void DrawMoon(int x, int y, int dd, int mm, int yy, char* hemisphere) {
#if 0
  int diameter = 38;
  float Xpos, Ypos, Rpos, Xpos1, Xpos2;
  gfx.setColor(EPD_BLACK);
  for (Ypos = 0; Ypos <= 45; Ypos++) {
    Xpos = sqrt(45 * 45 - Ypos * Ypos);
    // Draw dark part of moon
    double pB1x = (90   - Xpos) / 90 * diameter + x;
    double pB1y = (Ypos + 90) / 90   * diameter + y;
    double pB2x = (Xpos + 90) / 90   * diameter + x;
    double pB2y = (Ypos + 90) / 90   * diameter + y;
    double pB3x = (90   - Xpos) / 90 * diameter + x;
    double pB3y = (90   - Ypos) / 90 * diameter + y;
    double pB4x = (Xpos + 90) / 90   * diameter + x;
    double pB4y = (90   - Ypos) / 90 * diameter + y;
    //gfx.setColor(EPD_BLACK);
    hagl_draw_line(pB1x, pB1y, pB2x, pB2y, BLACK);
    hagl_draw_line(pB3x, pB3y, pB4x, pB4y, BLACK);
    // Determine the edges of the lighted part of the moon
    double Phase = NormalizedMoonPhase(dd, mm, yy);
    if (hemisphere == "south") Phase = 1 - Phase;
    Rpos = 2 * Xpos;
    if (Phase < 0.5) {
      Xpos1 = - Xpos;
      Xpos2 = (Rpos - 2 * Phase * Rpos - Xpos);
    }
    else {
      Xpos1 = Xpos;
      Xpos2 = (Xpos - 2 * Phase * Rpos + Rpos);
    }
    // Draw light part of moon
    double pW1x = (Xpos1 + 90) / 90 * diameter + x;
    double pW1y = (90 - Ypos) / 90  * diameter + y;
    double pW2x = (Xpos2 + 90) / 90 * diameter + x;
    double pW2y = (90 - Ypos) / 90  * diameter + y;
    double pW3x = (Xpos1 + 90) / 90 * diameter + x;
    double pW3y = (Ypos + 90) / 90  * diameter + y;
    double pW4x = (Xpos2 + 90) / 90 * diameter + x;
    double pW4y = (Ypos + 90) / 90  * diameter + y;
    //gfx.setColor(EPD_WHITE);
    hagl_draw_line(pW1x, pW1y, pW2x, pW2y, WHITE);
    hagl_draw_line(pW3x, pW3y, pW4x, pW4y, WHITE);
  }
  gfx.setColor(EPD_BLACK);
  gfx.drawCircle(x + diameter - 1, y + diameter, diameter / 2 + 1);
#endif
}
//#########################################################################################
char* MoonPhase(int d, int m, int y) {
#if 0
  const double Phase = NormalizedMoonPhase(d, m, y);
  int b = (int)(Phase * 8 + 0.5) % 8;
  if (b == 0) return "New";              // New; 0% illuminated
  if (b == 1) return "Waxing crescent";  // Waxing crescent; 25% illuminated
  if (b == 2) return "First quarter";    // First quarter; 50% illuminated
  if (b == 3) return "Waxing gibbous";   // Waxing gibbous; 75% illuminated
  if (b == 4) return "Full";             // Full; 100% illuminated
  if (b == 5) return "Waning gibbous";   // Waning gibbous; 75% illuminated
  if (b == 6) return "Third quarter";     // Last quarter; 50% illuminated
  if (b == 7) return "Waning crescent";  // Waning crescent; 25% illuminated
#endif
  return "";
}
//#########################################################################################
float MoonIllumination(int nDay, int nMonth, int nYear) { // calculate the current phase of the moon
  float age, phase, frac, days_since;
  long YY, MM, K1, K2, K3, JD;
  YY = nYear - floor((12 - nMonth) / 10);
  MM = nMonth + 9;
  if (MM >= 12) { MM = MM - 12;  }
  K1 = floor(365.25 * (YY + 4712));
  K2 = floor(30.6 * MM + 0.5);
  K3 = floor(floor((YY / 100) + 49) * 0.75) - 38;
  JD = K1 + K2 + nDay + 59;  //Julian day
  if (JD > 2299160) {JD = JD - K3; } //1582, Gregorian calendar
  days_since = JD - 2451550L; //since new moon on Jan. 6, 2000 (@18:00)
  phase = (days_since - 0.25) / 29.53059; //0.25 = correct for 6 pm that day
  phase -= floor(phase);  //phase in cycle
  age = phase * 29.53059;
  // calculate fraction full
  frac = (1.0 - cos(phase * 2 * PI)) * 0.5;
  if (frac > 1.0) frac = 2.0 - frac;  //illumination, accurate to about 5%
  return frac; //phase or age or frac, as desired
}

//#########################################################################################
void DrawWind(int x, int y, float angle, float windspeed){
    ESP_LOGI(TAG, "begin DrawWind");
  #define Cradius 15
  float dx = Cradius*cos((angle-90)*PI/180)+x;  // calculate X position
  float dy = Cradius*sin((angle-90)*PI/180)+y;  // calculate Y position
  arrow(x,y,Cradius-3,angle,10,12);   // Show wind direction on outer circle
  hagl_draw_circle(x,y,Cradius+2, BLACK);
  hagl_draw_circle(x,y,Cradius+3, BLACK);
  for (int m=0;m<360;m=m+45){
    dx = Cradius*cos(m*PI/180); // calculate X position
    dy = Cradius*sin(m*PI/180); // calculate Y position
    hagl_draw_line(x+dx,y+dy,x+dx*0.8,y+dy*0.8, BLACK);
  }
  //gfx.setTextAlignment(TEXT_ALIGN_CENTER);
  //gfx.setFont(ArialRoundedMTBold_14);
  //gfx.drawString(x,y+Cradius+3,WindDegToDirection(angle));
  //font_render_14.pixel_size++;
  render_text(WindDegToDirection(angle), &font_render_14, &driver, x,y+Cradius+3, 0, 0,0,0);
  //gfx.setFont(ArialMT_Plain_10);
  //gfx.drawString(x,y-Cradius-14,String(windspeed,1)+(Units=="M"?" m/s":" mph"));
  char message[20];
  sprintf(message,"%.1f%s",windSpeed, Units=='M'?" m/s":" mph");
  render_text(message, &font_render_10, &driver, x,y-Cradius-14, 0, 0,0,0);
  //gfx.setTextAlignment(TEXT_ALIGN_LEFT);
}
//#########################################################################################
char* WindDegToDirection(float winddirection){
    ESP_LOGI(TAG, "Begin WindDegToDirection");
  if (winddirection >= 348.75 || winddirection < 11.25)  return "N";
  if (winddirection >= 11.25  && winddirection < 33.75)  return "NNE";
  if (winddirection >= 33.75  && winddirection < 56.25)  return "NE";
  if (winddirection >= 56.25  && winddirection < 78.75)  return "ENE";
  if (winddirection >= 78.75  && winddirection < 101.25) return "E";
  if (winddirection >= 101.25 && winddirection < 123.75) return "ESE";
  if (winddirection >= 123.75 && winddirection < 146.25) return "SE";
  if (winddirection >= 146.25 && winddirection < 168.75) return "SSE";
  if (winddirection >= 168.75 && winddirection < 191.25) return "S";
  if (winddirection >= 191.25 && winddirection < 213.75) return "SSW";
  if (winddirection >= 213.75 && winddirection < 236.25) return "SW";
  if (winddirection >= 236.25 && winddirection < 258.75) return "WSW";
  if (winddirection >= 258.75 && winddirection < 281.25) return "W";
  if (winddirection >= 281.25 && winddirection < 303.75) return "WNW";
  if (winddirection >= 303.75 && winddirection < 326.25) return "NW";
  if (winddirection >= 326.25 && winddirection < 348.75) return "NNW";
  return "?";
}
//#########################################################################################
void arrow(int x, int y, int asize, float aangle, int pwidth, int plength){
    ESP_LOGI(TAG, "begin arrow");
  // x,y is the centre poistion of the arrow and asize is the radius out from the x,y position
  // aangle is angle to draw the pointer at e.g. at 45° for NW
  // pwidth is the pointer width in pixels
  // plength is the pointer length in pixels
  float dx = (asize-10)*cos((aangle-90)*PI/180)+x; // calculate X position
  float dy = (asize-10)*sin((aangle-90)*PI/180)+y; // calculate Y position
  float x1 = 0;         float y1 = plength;
  float x2 = pwidth/2;  float y2 = pwidth/2;
  float x3 = -pwidth/2; float y3 = pwidth/2;
  float angle = aangle*PI/180-135;
  float xx1 = x1*cos(angle)-y1*sin(angle)+dx;
  float yy1 = y1*cos(angle)+x1*sin(angle)+dy;
  float xx2 = x2*cos(angle)-y2*sin(angle)+dx;
  float yy2 = y2*cos(angle)+x2*sin(angle)+dy;
  float xx3 = x3*cos(angle)-y3*sin(angle)+dx;
  float yy3 = y3*cos(angle)+x3*sin(angle)+dy;
  hagl_fill_triangle(xx1,yy1,xx3,yy3,xx2,yy2, BLACK);
}
//#########################################################################################
void DrawPressureTrend(int x, int y, float pressure, char* slope){
    char text[20];
    sprintf(text, "%.2f%s", pressure, (Units=='M'?"mb":"in"));
  //gfx.drawString(90,47,String(pressure,1)+(Units=="M"?"mb":"in"));
    render_text(text, &font_render_10, &driver, 90,47,0, 0,0,0);
  x = x + 8;
  if      (slope == '+') {
    hagl_draw_line(x,  y,  x+4,y-4, GREEN);
    hagl_draw_line(x+4,y-4,x+8,y, GREEN);
  }
  else if (slope == '0') {
    hagl_draw_line(x+3,y-4,x+8,y, GREEN);
    hagl_draw_line(x+3,y+4,x+8,y, GREEN);
  }
  else if (slope == '-') {
    hagl_draw_line(x,  y,  x+4,y+4, GREEN);
    hagl_draw_line(x+4,y+4,x+8,y, GREEN);
  }
}
//#########################################################################################
void DisplayWXicon(int x, int y, char* iconName, bool LargeSize){
//  iconName.toLowerCase(); IconName.trim();
//    for(int i = 0; iconName[i]; i++){
//      iconName[i] = tolower(iconName[i]);
//    }
//  Serial.println(IconName);
    ESP_LOGI("DisplayWXicon", "iconName=%s ",iconName);
  if      (!strcmp(iconName, "01d") || !strcmp(iconName, "01n"))
      if (LargeSize) Sunny(x,y,Large); else Sunny(x,y,Small);
  else if (!strcmp(iconName, "02d") || !strcmp(iconName, "02n"))
      if (LargeSize) MostlySunny(x,y,Large); else MostlySunny(x,y,Small);
  else if (!strcmp(iconName, "03d") || !strcmp(iconName, "03n"))
      if (LargeSize) Cloudy(x,y,Large); else Cloudy(x,y,Small);
  else if (!strcmp(iconName, "04d") || !strcmp(iconName, "04n"))
      if (LargeSize) MostlySunny(x,y,Large); else MostlySunny(x,y,Small);
  else if (!strcmp(iconName, "09d") || !strcmp(iconName, "09n"))
      if (LargeSize) ChanceRain(x,y,Large); else ChanceRain(x,y,Small);
  else if (!strcmp(iconName, "10d") || !strcmp(iconName, "10n"))
      if (LargeSize) Rain(x,y,Large); else Rain(x,y,Small);
  else if (!strcmp(iconName, "11d") || !strcmp(iconName, "11n"))
      if (LargeSize) Tstorms(x,y,Large); else Tstorms(x,y,Small);
  else if (!strcmp(iconName, "13d") || !strcmp(iconName, "13n"))
      if (LargeSize) Snow(x,y,Large); else Snow(x,y,Small);
  else if (!strcmp(iconName, "50d"))
      if (LargeSize) Haze(x,y-5,Large); else Haze(x,y,Small);
  else if (!strcmp(iconName, "50n"))
      if (LargeSize) Fog(x,y-5,Large); else Fog(x,y,Small);
  else if (!strcmp(iconName, "probrain"))
      if (LargeSize) ProbRain(x,y,Large); else ProbRain(x,y,Small);
  else if (LargeSize) Nodata(x,y,Large); else Nodata(x,y,Small);
}

int currentHour()
{
    time_t now;
    time(&now);
    char strftime_buf[64];
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%D %T", &timeinfo);
    ESP_LOGI(TAG, "The current local date/time is: %s, hour=%d", strftime_buf, timeinfo.tm_hour);
    return timeinfo.tm_hour;
}
int getHour(time_t time)
{
    time_t now = time;
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    return timeinfo.tm_hour;
}

void format_time(char *dest, time_t time)
{
    time_t now = time;
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    char strftime_buf[64];
    strftime(strftime_buf, sizeof(strftime_buf),"%H:%M", &timeinfo);
    strcpy(dest, strftime_buf);
}

time_t convertTime(long dt)
{
    time_t now = (time_t)dt + tzOffset;
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    char strftime_buf[64];

    //time(&now);
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "The date/time is: %s", strftime_buf);
    return now;
}

void time_cb(struct timeval *tv)
{
    // NTP time received
    time_t now;
    char strftime_buf[64];
    struct tm timeinfo;

    time(&now);
    // Set timezone to China Standard Time
    setenv("TZ", "EST+5EDT", 1);
    tzset();

    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "The current date/time in Louisville is: %s", strftime_buf);

//    if(!time_display_started)
//    {
//     xTaskCreatePinnedToCore(time_display, "Switch", 8092, NULL, 2, NULL, 1);
//     time_display_started = true;
//    }

    UpdateLocalTime();

    xTaskCreatePinnedToCore(main_task, "Demo", 8092, NULL, 1, NULL, 1);
}

void time_display(void *params)
{
    color_t green = hagl_color(0, 255, 0);
    char16_t message[128];
    swprintf(message, sizeof(message), u"%s    ", ipAddr);
    hagl_set_clip_window(0, 0, DISPLAY_WIDTH - 1, DISPLAY_HEIGHT - 1);
    hagl_put_text(message, 4, 4, green, font6x9);

    // display time every second
    color_t yellow = hagl_color(255, 255, 0);
    // Block for 1000ms.
    const TickType_t xDelay = 1000 / portTICK_PERIOD_MS;

    // start sntp time
    if(!sntp_time_started)
    {
        start_NTP();
    }

    for( ;; )
    {
        // Simply toggle the LED every 500ms, blocking between each toggle.
        time_t now;
        char16_t message[128];
        char strftime_buf[64];
        struct tm timeinfo;

        time(&now);
        // Set timezone to Eastern Standard Time
        setenv("TZ", "EST5EDT", 1);
        tzset();

        localtime_r(&now, &timeinfo);
        strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
        swprintf(message, sizeof(message), u"%s    ", strftime_buf);
        hagl_set_clip_window(0, 0, DISPLAY_WIDTH - 1, DISPLAY_HEIGHT - 1);
//        hagl_put_text(message, 4, 21, yellow, fnt9x18b);
        //ESP_LOGI(TAG, "%s", strftime_buf);
        int min = timeinfo.tm_min;
        if(min%5 ==0 && timeinfo.tm_sec == 0)
         xTaskCreatePinnedToCore(main_task, "Demo", 8092, NULL, 1, NULL, 1);
        vTaskDelay( xDelay );
    }
}

void wifi_init_softap(void *params)
{
    ESP_LOGI(TAG, "Heap wifi_init_softap enter: %d", esp_get_free_heap_size());
    s_wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL) );

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_ESP_WIFI_SSID,
            .password = CONFIG_ESP_WIFI_PASS,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");
    ESP_LOGI(TAG, "connect to ap SSID:%s password:%s",
             CONFIG_ESP_WIFI_SSID, CONFIG_ESP_WIFI_PASS);

    vTaskDelete(NULL);
}

void app_main()
{
    heap_caps_enable_nonos_stack_heaps();

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "SDK version: %s", esp_get_idf_version());
    ESP_LOGI(TAG, "Heap when starting: %d", esp_get_free_heap_size());
    event = xEventGroupCreate();

    /* Save the backbuffer pointer so we can later read() directly into it. */
    hagl_init();
    hagl_driver_t display = {
            .display_width = DISPLAY_WIDTH,
            .display_height = DISPLAY_HEIGHT,
            .buffer_size = BUFFER_SIZE * DISPLAY_WIDTH	// 2 buffers with 20 lines
    };

    setup_Gpio();

    ESP_ERROR_CHECK(font_face_init(&font_face, ttf_start, ttf_end - ttf_start - 1));

#ifdef HAGL_HAL_USE_BUFFERING
    xTaskCreatePinnedToCore(flush_task, "Flush", 4096, NULL, 1, NULL, 0);
#endif
    xTaskCreatePinnedToCore(wifi_init_softap, "wifi", 8092, NULL, 2, NULL, 1);
}

