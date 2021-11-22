#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "UbidotsEsp32Mqtt.h"
#include "SSD1306Wire.h" // legacy include: `#include "SSD1306.h"`

SSD1306Wire  display(0x3c, 18, 19);  //18=SDK  19=SCK  As per labeling on ESP32 DevKit
#define SEALEVELPRESSURE_HPA (1013.25)

char buffT[6];
char buffA[6];
char buffH[6];
char buffP[6];
Adafruit_BME280 bme;

const char *UBIDOTS_TOKEN = "BBFF-OGwA21Z6uhoggc3wiUHnvhLUPbnRsh";  // Put here your Ubidots TOKEN
const char *WIFI_SSID = "Eirik sin iPhone";      // Put here your Wi-Fi SSID
const char *WIFI_PASS = "1234567890";      // Put here your Wi-Fi password
const char *DEVICE_LABEL = "esp32";   // Replace with the device label to subscribe to
const char *VARIABLE_Temperature = "temperatur"; // Replace with your variable label to subscribe to
const char *VARIABLE_Humidity = "humidity"; // Replace with your variable label to subscribe to
const char *VARIABLE_Altitude = "altitude"; // Replace with your variable label to subscribe to
const char *VARIABLE_Pressure = "pressure"; // Replace with your variable label to subscribe to


const int PUBLISH_FREQUENCY = 5000; // Update rate in milliseconds

unsigned long timer;
unsigned long timer1;

Ubidots ubidots(UBIDOTS_TOKEN);

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.println("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.println((char)payload[i]);
    if (topic = "temperatur"){
      *buffT = payload[i];
    }
    if (topic = "humidity"){
      *buffH = payload[i];
    }
    if (topic = "altitude"){
      *buffA = payload[i];
    }
    if (topic = "pressure"){
      *buffP = payload[i];
    }
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  ubidots.connectToWifi(WIFI_SSID, WIFI_PASS);
  ubidots.setCallback(callback);
  ubidots.setup();
  ubidots.reconnect();
  timer = millis();
  bool status;
  status = bme.begin(0x76);  
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
}

void loop() { 
  if (!ubidots.connected()){
    ubidots.reconnect();
  }
  publis();
  screen();
  ubidots.loop();
}
void publis(){
  if (abs(millis() - timer) > PUBLISH_FREQUENCY){
    float Temperature = bme.readTemperature();
    float Pressure = (bme.readPressure() / 100.0F);
    float Altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
    float Humidity = bme.readHumidity();
    if (!isnan(Temperature)) {
      ubidots.add(VARIABLE_Temperature, Temperature); // Insert your variable Labels and the value to be sent
    }
    if (!isnan(Humidity)) {
        ubidots.add(VARIABLE_Humidity, Humidity); // Insert your variable Labels and the value to be sent 
    }
    if (!isnan(Temperature)) {
      ubidots.add(VARIABLE_Altitude, Altitude); // Insert your variable Labels and the value to be sent
    }
    if (!isnan(Humidity)) {
        ubidots.add(VARIABLE_Pressure, Pressure); // Insert your variable Labels and the value to be sent 
    }
    ubidots.publish(DEVICE_LABEL);
    timer = millis();
  }
}
void screen() {
  if (abs(millis() - timer1) > PUBLISH_FREQUENCY){
  ubidots.subscribeLastValue(DEVICE_LABEL, VARIABLE_Temperature);
  ubidots.subscribeLastValue(DEVICE_LABEL, VARIABLE_Humidity);
  ubidots.subscribeLastValue(DEVICE_LABEL, VARIABLE_Altitude);
  ubidots.subscribeLastValue(DEVICE_LABEL, VARIABLE_Pressure);
  display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0, "Sensor");
      
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 20, "Temperature");
    display.drawString(75, 20, String(buffT));

    display.setFont(ArialMT_Plain_10);
    display.drawString(100, 20, "*C");

    display.drawString(0, 30, "Humidity");
    display.setFont(ArialMT_Plain_10);
    display.drawString(75, 30, String(buffH));
    display.setFont(ArialMT_Plain_10);
    display.drawString(100, 30, "%");

    display.drawString(0, 40, "Altitude");
    display.setFont(ArialMT_Plain_10);
    display.drawString(75, 40, String(buffA));
    display.setFont(ArialMT_Plain_10);
    display.drawString(100, 40, "m");

    display.drawString(0, 50, "Pressure");
    display.setFont(ArialMT_Plain_10);
    display.drawString(75, 50, String(buffP));
    display.setFont(ArialMT_Plain_10);
    display.drawString(100, 50, "hPa");
  display.display();
  timer1 = millis();
  }
}
