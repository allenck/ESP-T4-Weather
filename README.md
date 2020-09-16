# ESP-T4-Weather
Display current weather an forecast on TTGO T4 display.

This is an adaption of the application [ESP32_OWM_Current_Forecast_29_epaper_v7](https://github.com/G6EJD/ESP32-e-Paper-Weather-Display) for the TTGO T4 display.


Assuming that you have a Projects folder, perform the following"
```
cd ~/Projects 
git clone https://github.com/allenck/ESP-T4-Weather.git
cd ESP-T4-Weather
git submodule update --recursive
cp sdkconfig.ttgo-t4-v13 sdkconfig
git_idf # or [see "getting Started")](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html?highlight=get_idf#get-started-set-up-env)
cmake .
idf.py menuconfig
```
In the menuconfig page, select "Component config".

![Menuconfig](/images/menuconfig1.png)

Next, select "ESP-T4-Weather Application parameters" then on the following page, enter the appropriate values:
![Menuconfig](/images/menuconfig2.png)
1. Your WiFi's SSID and Password. 
2. Number of times to retry logging on to WiFi.
3. Your geographic location's latitude and longitude.
4. The name of your location to be displayed. 
5. Whether you want values like temperature, wind speed, pressur displayed in Metric or Imperial units.
6. Your API key that you obtained from regitering at [openweathermap.org](https://rapidapi.com/blog/lp/openweathermap/?utm_source=google&utm_medium=cpc&utm_campaign=Alpha_105406266670&utm_term=openweathermap%20api%20key_e&gclid=CjwKCAjw74b7BRA_EiwAF8yHFJD58MeQJv75tIp4J3_nOIhGJLisV1r8xSr8woBkAylJIZeBRRui-hoCoB0QAvD_BwE).
Save and exit menuconfig.

If you use Qt Creator, a Qt Cteator .pro file, "ESP-T4-Weather.pro" is provided.




