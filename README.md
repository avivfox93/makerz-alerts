# makerz-alerts

Setup:

NodeMCU ESP8266 powered via USB (I used Espessiff) 
5V RGB LED strip (something that plays nice with NeoPixel). 
    - Data pin connected to D3 on ESP8266 
    - GND connected to ESP8266
    - Power: I powered through the ESP for quick testing. While it works fine, it surely is not the best practice 


Expected behaviour: 
    - communication is with fade-in-and-out and with blinking of LED number 3 (the 4th from the controller) 
    - startup -> white fade indicates status is ok 
    - wifi connection: 
        - if not connected, an access point will be available for congiguration. SSID: Makers_Alerts_Wifi
        - after connection, green fade indicates succesfull connection 
    - heartbeat sent to MQTT - green blinking in 4th LED every ~5 seconds 
    - status - ok - green blinking of the while strip every ~60 seconds 
    - alerts - red fade 
    - warnings ( = alerts older than 5 minutes and youner than 30) - yellow fade (not sure if that's needed)

