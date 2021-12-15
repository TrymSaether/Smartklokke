#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "UbidotsEsp32Mqtt.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
const int Read_Speed = 5000;
const int Publish_Frquency = 5000;
const int Subscribe_Speed = 5000;
const int Sealevl_Pressure = 1013.25;
unsigned long Timer_Screen;
unsigned long Timer_Publish;
unsigned long Timer_Subscribe;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Adafruit_BME280 bme;


const char *UBIDOTS_TOKEN = "BBFF-OGwA21Z6uhoggc3wiUHnvhLUPbnRsh";  // Put here your Ubidots TOKEN
const char *WIFI_SSID = "Eirik sin iPhone";      // Put here your Wi-Fi SSID
const char *WIFI_PASS = "1234567890";      // Put here your Wi-Fi password
const char *DEVICE_LABEL = "esp32";   // Replace with the device label to subscribe to
const char *VARIABLE_Temperature = "temperatur"; // Replace with your variable label to subscribe to
const char *VARIABLE_Humidity = "humidity"; // Replace with your variable label to subscribe to
const char *VARIABLE_Altitude = "altitude"; // Replace with your variable label to subscribe to
const char *VARIABLE_Pressure = "pressure"; // Replace with your variable label to subscribe to
Ubidots ubidots(UBIDOTS_TOKEN);
char *Topic_Temperature = "/v2.0/devices/esp32/temperatur/lv";
char *Topic_Humidity = "/v2.0/devices/esp32/humidity/lv";
char *Topic_Altitude = "/v2.0/devices/esp32/altitude/lv";
char *Topic_Pressure = "/v2.0/devices/esp32/pressure/lv";
String(Sensor_Temperature);
String(Sensor_Humidity);
String(Sensor_Altitude);
String(Sensor_Pressure);

        
void callback(char *topic, byte *payload, unsigned int length){

  if (strcmp(topic, Topic_Temperature)==0){
    Sensor_Temperature = (char *)payload;
  }
  if (strcmp(topic, Topic_Humidity)==0){
    Sensor_Humidity = (char *)payload;
  }
  if (strcmp(topic, Topic_Altitude)==0){
    Sensor_Altitude = (char *)payload;
  }
  if (strcmp(topic, Topic_Pressure)==0){
    Sensor_Pressure = (char *)payload;
  }
}
void publis(){
    if (abs(millis() - Timer_Publish) > Publish_Frquency){
        float Temperature = bme.readTemperature();
        float Pressure = (bme.readPressure()/100.0F);
        float Altitude = bme.readAltitude(Sealevl_Pressure);
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
        Timer_Publish = millis();
    }
}
void subscribe(){
  if (abs(millis() - Timer_Subscribe) > Publish_Frquency){
    ubidots.subscribeLastValue(DEVICE_LABEL, VARIABLE_Temperature);
    ubidots.subscribeLastValue(DEVICE_LABEL, VARIABLE_Humidity);
    ubidots.subscribeLastValue(DEVICE_LABEL, VARIABLE_Altitude);
    ubidots.subscribeLastValue(DEVICE_LABEL, VARIABLE_Pressure);
  }
}

void screen_online(){
    if (abs(millis() - Timer_Screen) > Read_Speed){
        display.setCursor(0,0);
        display.clearDisplay();
        display.print("Sensor");
        display.setCursor(0,20); display.print("Temperature:"); display.print(Sensor_Temperature); display.print(" *C");
        display.setCursor(0,30); display.print("Humidity:   "); display.print(Sensor_Humidity); display.print(" %");
        display.setCursor(0,40); display.print("Altitude:   "); display.print(Sensor_Altitude); display.print(" m");
        display.setCursor(0,50); display.print("Pressure:   "); display.print(Sensor_Pressure); display.print(" hPa");
        display.display();
        Timer_Screen = millis();
    }
}

void screen_offline(){
  float Temperature = bme.readTemperature();
  float Pressure = (bme.readPressure()/100.0F);
  float Altitude = bme.readAltitude(Sealevl_Pressure);
  float Humidity = bme.readHumidity();
  if (abs(millis() - Timer_Screen) > Read_Speed){
        display.setCursor(0,0);
        display.clearDisplay();
        display.print("Sensor");
        display.setCursor(0,20); display.print("Temperature:"); display.print(Temperature); display.print(" *C");
        display.setCursor(0,30); display.print("Humidity:   "); display.print(Humidity); display.print(" %");
        display.setCursor(0,40); display.print("Altitude:   "); display.print(Altitude); display.print(" m");
        display.setCursor(0,50); display.print("Pressure:   "); display.print(Pressure); display.print(" hPa");
        display.display();
        Timer_Screen = millis();
  }
}

void Clock(){
  
}
void setup() {
  Serial.begin(115200);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  
  bool status = bme.begin(0x76);  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
 
  delay(2000);
  display.clearDisplay();
  ubidots.connectToWifi(WIFI_SSID, WIFI_PASS);
  ubidots.setCallback(callback);
  ubidots.setup();
  ubidots.reconnect();
  Timer_Screen = millis();
  Timer_Publish = millis();
  Timer_Subscribe = millis();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime();
}

void loop() {
    if (!ubidots.connected()){
      screen_offline();
      ubidots.reconnect();
    }
    screen_online();
    publis();
    subscribe();
    ubidots.loop();
}
