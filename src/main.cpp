#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <DoubleResetDetector.h>
#include <PubSubClient.h>
#include <FastLED.h>

// DRD
#define DRD_TIMEOUT 10 // timeout for 2nd click (double reset)
#define DRD_ADDRESS 0  // RTC Memory Address for the DoubleResetDetector

// WiFi
#define AP_NAME "makers-alerts-AP"
DoubleResetDetector drd(DRD_TIMEOUT, DRD_ADDRESS);
WiFiManager wifiManager;
WiFiClient espClient;

// MQTT
#define TOPIC_ALERT "ra_alert"
#define TOPIC_WARNING "ra_warning"
#define TOPIC_ALERT_COUNT "ra_alert/count"
#define TOPIC_WARNING_COUNT "ra_warning/count"
#define TOPIC_HEARTBEAT "client_heartbeat"
PubSubClient mqttClient(espClient);
const char *mqtt_server = "mqtt.makerspace.co.il";
const int mqtt_port = 1883;
const char *topic_sub = "ra_alert";

// Heartbeat print
#define HEARTBEAT_MILLIS 5000
#define HEARTBEAT_MSG_BUFFER_SIZE (50)
char msg[HEARTBEAT_MSG_BUFFER_SIZE];
unsigned long lastMsg = 0;
long int heartbeatValue = 0;

// Leds

#ifndef NUM_LEDS
  #warning NUM_LEDS not provided as a build flag (in platform.ini). using default value.
#define NUM_LEDS 300
#endif // NUM_LEDS

#ifndef PIN_LEDS
  #warning PIN_LEDS not provided as a build flag (in platform.ini). using default value.
#define PIN_LEDS 3
#endif // PIN_LEDS

#define BRIGHTNESS 128
#define ALERT_COLOR_WAIT_TIME 500
CRGB leds[NUM_LEDS];

void leds_initStrip()
{
  FastLED.addLeds<WS2812B, PIN_LEDS, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
}

void leds_turnOff()
{
  FastLED.clear();
  FastLED.show();
}

void leds_fadeOut()
{
  int fadeAmount = 1;
  int scale = 255;
  while (scale >= 0)
  {
    nscale8(leds, NUM_LEDS, scale);
    FastLED.show();
    delay(50);
    scale = scale - fadeAmount;
  }
}

void leds_rainbow(uint8_t hue)
{
  fill_rainbow(leds, NUM_LEDS, hue, 255 / NUM_LEDS);
  FastLED.show();
}

void leds_wifiConnected()
{
  fill_solid(leds, NUM_LEDS, CRGB::Green); // https://github.com/FastLED/FastLED/wiki/Pixel-reference#predefined-colors-list
  FastLED.show();
  leds_fadeOut();
}

void leds_mqttConnected()
{
  fill_solid(leds, NUM_LEDS, CRGB::HotPink);
  FastLED.show();
  leds_fadeOut();
}

void leds_redAlert()
{
  fill_solid(leds, NUM_LEDS, CRGB::Red);
  FastLED.show();
  delay(ALERT_COLOR_WAIT_TIME);
  leds_fadeOut();
}

void leds_redAlertWarning()
{
  fill_solid(leds, NUM_LEDS, CRGB::Yellow);
  FastLED.show();
  delay(ALERT_COLOR_WAIT_TIME);
  leds_fadeOut();
}

void wifi_connectOrAP()
{
  wifiManager.autoConnect(AP_NAME);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  randomSeed(micros());
  Serial.println("");
  Serial.println("WiFi connected");

  leds_wifiConnected();
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void mqtt_eventCallback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Event received [topic ");
  Serial.print(topic);
  Serial.print("]: ");

  String strPayload = " ";
  String Digits = "";
  String strReversedPayload = " ";

  for (int i = 0; i < length; i++)
  {
    char inChar = (char)payload[i];
    strPayload += inChar;
    strReversedPayload += inChar;
    if (isDigit(inChar))
    {
      Digits += inChar;
    }
  }
  strReversedPayload += "*";
  strPayload += " ";

  for (int i = 0; i < strReversedPayload.length(); i++)
  {
    strReversedPayload.setCharAt(strReversedPayload.length() - i - 1, strPayload[i]);
  }

  if (String(topic).indexOf(TOPIC_ALERT_COUNT) != -1)
  {
    int alertsCount = Digits.toInt();
    if (alertsCount > 0)
    {
      leds_redAlert();
    }
  }
  //if we alert in red, no need to mention warnings as well
  else
  {
    if (String(topic).indexOf(TOPIC_WARNING_COUNT) != -1)
    {
      int warningcount = Digits.toInt();
      if (warningcount > 0)
      {
        leds_redAlertWarning();
      }
    }
  }
  Serial.println(strPayload);
  Serial.println(strReversedPayload);
  Serial.println(Digits);
}

void mqtt_init()
{
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqtt_eventCallback);
  Serial.println("MQTT client set-up");
}

String mqtt_generateClientId()
{
  return "client8266-" + String(WiFi.macAddress());
}

void mqtt_reconnect()
{
  while (!mqttClient.connected())
  {
    Serial.println("trying to connect to MQTT...");

    if (mqttClient.connect(mqtt_generateClientId().c_str()))
    {
      Serial.println("mqtt connected");
      mqttClient.subscribe(TOPIC_ALERT_COUNT);
      mqttClient.subscribe(TOPIC_WARNING_COUNT);
      mqttClient.subscribe(TOPIC_ALERT);
      mqttClient.subscribe(TOPIC_WARNING);
      leds_mqttConnected();
      // Notify server
      //  mqttClient.publish(TOPIC_HEARTBEAT, "device connected");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}
void drd_setup()
{
  if (drd.detectDoubleReset())
  {
    Serial.println("Double Reset Detected - erasing Config, restarting");
    digitalWrite(LED_BUILTIN, LOW);
    // wifiManager.resetSettings();
  }
  else
  {
    Serial.println("No Double Reset Detected");
    digitalWrite(LED_BUILTIN, HIGH);
  }
}

void utils_printHeartbeat()
{
  unsigned long now = millis();
  if (now - lastMsg > HEARTBEAT_MILLIS)
  {
    lastMsg = now;
    ++heartbeatValue;
    snprintf(msg, HEARTBEAT_MSG_BUFFER_SIZE, "heartbeat #%ld %s", heartbeatValue, mqtt_generateClientId().c_str());
    Serial.println(msg);
  }
}

void setup()
{
  Serial.begin(115200);
  leds_initStrip();
  wifi_connectOrAP();
  mqtt_init();
}

void loop()
{
  drd.loop();
  if (!mqttClient.connected())
  {
    mqtt_reconnect();
  }
  mqttClient.loop();
  utils_printHeartbeat();
}
