#include <Arduino.h>
#include <PubSubClient.h>
#include "UbidotsEsp32Mqtt.h"
#include "ICM_20948.h"

// - Constants -
const char *UBIDOTS_TOKEN = "BBFF-eHgsVCIJbnSJQxJZ7wHKPZ3VQOuf6P";  // Put here your Ubidots TOKEN
const char *WIFI_SSID = "Trym sin iPhone";      // Put here your Wi-Fi SSID
const char *WIFI_PASS = "eplepai1";      // Put here your Wi-Fi password
const char *DEVICE_LABEL = "smartklokke";   // Put here your Device label to which data  will be published
const char *VARIABLE_LABEL = "knapp"; // Put here your Variable label to which data  will be published
const int PUBLISH_FREQUENCY = 5000; // Update rate in milliseconds
const int button = 32;
unsigned long timer;

Ubidots ubidots(UBIDOTS_TOKEN);

/****************************************
 * Auxiliar Functions
 ****************************************/

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

/****************************************
 * Main Functions
 ****************************************/

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  // ubidots.setDebug(true);  // uncomment this to make debug messages available
  ubidots.connectToWifi(WIFI_SSID, WIFI_PASS);
  ubidots.setCallback(callback);
  ubidots.setup();
  ubidots.reconnect();

  pinMode(button,INPUT);
  timer = millis();
  
}

void loop()
{
  int val1 = digitalRead(button);
  // put your main code here, to run repeatedly:
  if (!ubidots.connected())
  {
    ubidots.reconnect();
  }
  if (abs(millis() - timer) > PUBLISH_FREQUENCY) // triggers the routine every 5 seconds
  {
    Serial.println(val1);
    ubidots.add(VARIABLE_LABEL, val1); // Insert your variable Labels and the value to be sent
    ubidots.publish(DEVICE_LABEL);
    timer = millis();
  }
  ubidots.loop();
}