#include <Arduino.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <UbidotsEsp32Mqtt.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include "time.h"
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // Bredde på OLED skjerm i pixels
#define SCREEN_HEIGHT 64 // Høyde på OLED skjerm i pixels
#define BUTTON_PIN 15   // Tinkoblingspinne for knapp
const int Read_Speed = 5000; // Frekvens på avlesning av bme280
const int Publish_Frquency = 5000;  // frekvens for opplastning til Ubidots
const int Subscribe_Speed = 5000;   // frekvens for avlesning av Ubidots
const int Sealevl_Pressure = 1013.25;   // kalibrering av trykk
const int Button_Read_Speed = 1000;     // Sperre for å unngå flere avlesninger av et knappetrykk
unsigned long Timer_Screen;     // oppdateringshastighet av skjerm
unsigned long Timer_Publish;    // tid siden siste publisering
unsigned long Timer_Subscribe;  // tid siden siste avlesning av Ubidost
unsigned long Button_Last;      // tid siden knapp ble trykket
int count = 0;                  // variabel for å bytte mellom skjermsider

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1); // Setter parametere for skjerm
Adafruit_BME280 bme;    // I2C

const char *UBIDOTS_TOKEN = "BBFF-OGwA21Z6uhoggc3wiUHnvhLUPbnRsh"; // Ubidots TOKEN
const char *WIFI_SSID = "Eirik sin iPhone";                        // Wi-Fi SSID
const char *WIFI_PASS = "1234567890";                              // Wi-Fi password
const char *DEVICE_LABEL = "esp32";                                // Device label på Ubidots vi ønsker å subscribe til
const char *VARIABLE_Temperature = "temperatur";    // variable label vi ønsker å koble oss til
const char *VARIABLE_Humidity = "humidity";                     
const char *VARIABLE_Altitude = "altitude";                       
const char *VARIABLE_Pressure = "pressure";                        
const char *VARIABLE_Steps = "steps";                              
const char *ntpServer = "pool.ntp.org"; // Nettside vi bruker for å hente klokkeslett          
const long gmtOffset_sec = 0;   // for å stille klokken til en annen tidssone
const int daylightOffset_sec = 0;    // for å få sommertid settes denne til 3600
Ubidots ubidots(UBIDOTS_TOKEN);
char *Topic_Temperature = "/v2.0/devices/esp32/temperatur/lv"; // for å lese av hvilke topic som kommer
char *Topic_Humidity = "/v2.0/devices/esp32/humidity/lv";
char *Topic_Altitude = "/v2.0/devices/esp32/altitude/lv";
char *Topic_Pressure = "/v2.0/devices/esp32/pressure/lv";
String(Sensor_Temperature); // lagringsplass for payload
String(Sensor_Humidity);
String(Sensor_Altitude);
String(Sensor_Pressure);
int lastState = LOW; // på/av knapp før
int currentState;    // på/av knapp nå
struct tm timeinfo; // tid og dato brutt opp som komponeter

// - Deklarasjon -
// ICM-20948
Adafruit_ICM20948 icm;
sensors_event_t acc;
sensors_event_t temp;
sensors_event_t mag;
sensors_event_t gyro;

// Akselerasjon
float x, y, z;
static float currentx = 0;
static float currenty = 0;
static float currentz = 0;
static float currentVectorMag = 0;

const float xErr = 0.80; // Sensorkorigeringsverdier.
const float yErr = 0.50;
const float zErr = 10.22;
const float threshold = 0.1; // Grense for stegakselerasjon
const int maxThreshold = 20.5;
float vectorMag;

// Diverse
char str[50];
byte buff[10];
static unsigned long stepSum = 0;
int deltaStepSum = 0;
bool statusIcm;
bool statusBme;
// - Funksjoner -
void setupICM()
{
    statusIcm = icm.begin_I2C(0x68);
    if (!statusIcm)
    {
        Serial.println("Finner ikke ICM 20948 sensor!");
        while (1)
            ;
    }
    Serial.println("ICM20948 Funnet!");
    Serial.println("OK");
    icm.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
    icm.setGyroRateDivisor(255);
    icm.setMagDataRate(AK09916_MAG_DATARATE_10_HZ);
    icm.setAccelRange(ICM20948_ACCEL_RANGE_2_G);
    icm.setAccelRateDivisor(4095);
}
void Clock()    // for å vise klokken på skjermen
{
    if (!getLocalTime(&timeinfo))   // skjekker om avlesning av klokken ble gjennomført riktig
    {
        Serial.println("Failed to obtain time");
        return;
    }
    display.clearDisplay();  // sletter alt på skjermen
    if (!ubidots.connected())   // skjekker tilkobling til skjermen
    {
        display.setTextSize(1); // tekststørrelse på skjermen
        display.setCursor(0, 0);    // plasering av første pixel på skjermen
        display.print("        Offline");   // printer til skjerm
    }
    display.setTextSize(2); // tekststørrelse på skjermen
    display.setCursor(0, 20);    // plasering av første pixel på skjermen
    display.print(&timeinfo, " %H:%M:%S");  // printer klokkeslett til skjerm
    display.display();  // for å vise alt som har blitt printet til skjermen
}
void callback(char *topic, byte *payload, unsigned int length)  // for å mota data fra Ubidots
{

    if (strcmp(topic, Topic_Temperature) == 0) // skjekker topic
    {   
        Sensor_Temperature = String((char *)payload);   // lagrer payloaden
    }
    if (strcmp(topic, Topic_Humidity) == 0)
    {
        Sensor_Humidity = String((char *)payload);
    }
    if (strcmp(topic, Topic_Altitude) == 0)
    {
        Sensor_Altitude = String((char *)payload);
    }
    if (strcmp(topic, Topic_Pressure) == 0)
    {
        Sensor_Pressure = String((char *)payload);
    }
}
void publis()   // for publisering
{
    if ((abs(millis() - Timer_Publish) > Publish_Frquency) && (ubidots.connected())) // Skjekker tilkobling til ubidots og hvor lenge siden siste opplastning
    {
        float Temperature = bme.readTemperature();          // leser av bme280 sensor
        float Pressure = (bme.readPressure() / 100.0F);
        float Altitude = bme.readAltitude(Sealevl_Pressure);
        float Humidity = bme.readHumidity();
        if (!isnan(Temperature))    // skjekker at avlesning ikke er en NaN
        {
            ubidots.add(VARIABLE_Temperature, Temperature); // klargjør data til publisering til ubidots
        }
        if (!isnan(Humidity))
        {
            ubidots.add(VARIABLE_Humidity, Humidity); 
        }
        if (!isnan(Temperature))
        {
            ubidots.add(VARIABLE_Altitude, Altitude); 
        }
        if (!isnan(Humidity))
        {
            ubidots.add(VARIABLE_Pressure, Pressure); 
        }
        if (!isnan(stepSum))
        {
            ubidots.add(VARIABLE_Steps, stepSum); 
        }
        ubidots.publish(DEVICE_LABEL);  // publiserer til det som er klargjort for opplasting
        Timer_Publish = millis();       // setter tid for forrige opplasting
    }
}
void subscribe() // for å hente data fra ubidots
{
    if ((abs(millis() - Timer_Subscribe) > Publish_Frquency) && (ubidots.connected())) // Skjekker tilkobling til ubidots og hvor lenge siden siste opplastning
    {
        ubidots.subscribeLastValue(DEVICE_LABEL, VARIABLE_Temperature); // sender forespørsel om data til ubidots
        ubidots.subscribeLastValue(DEVICE_LABEL, VARIABLE_Humidity);
        ubidots.subscribeLastValue(DEVICE_LABEL, VARIABLE_Altitude);
        ubidots.subscribeLastValue(DEVICE_LABEL, VARIABLE_Pressure);
        Timer_Subscribe = millis(); // setter tid for forrige forspørsel
    }
}

void screen_online() // Forå vise data fra bme280 på skjerm når online 
{
    if (abs(millis() - Timer_Screen) > Read_Speed) // oppdaterings hastighet av skjerm 
    {
        display.setCursor(0, 0);
        display.clearDisplay();
        display.setTextSize(1);
        display.print("Sensor");
        display.setCursor(0, 20);
        display.print("Temperature:");
        display.print(Sensor_Temperature); // data fra ubidots
        display.print("*C");
        display.setCursor(0, 30);
        display.print("Humidity:  ");
        display.print(Sensor_Humidity);
        display.print("%");
        display.setCursor(0, 40);
        display.print("Altitude:  ");
        display.print(Sensor_Altitude);
        display.print("m");
        display.setCursor(0, 50);
        display.print("Pressure:  ");
        display.print(Sensor_Pressure);
        display.print("hPa");
        display.display();
        Timer_Screen = millis();
    }
}



void screen_offline() // Forå vise data fra bme280 på skjerm når offline
{
    if (abs(millis() - Timer_Screen) > Read_Speed)
    {
        float Temperature = bme.readTemperature();
        float Pressure = (bme.readPressure() / 100.0F);
        float Altitude = bme.readAltitude(Sealevl_Pressure);
        float Humidity = bme.readHumidity();
        display.setCursor(0, 0);
        display.clearDisplay();
        display.setTextSize(1);
        display.print("Sensor  ");
        display.print("Offline");
        display.setCursor(0, 20);
        display.print("Temperature:");
        display.print(Temperature);
        display.print("C");
        display.setCursor(0, 30);
        display.print("Humidity:  ");
        display.print(Humidity);
        display.print("%");
        display.setCursor(0, 40);
        display.print("Altitude:  ");
        display.print(Altitude);
        display.print("m");
        display.setCursor(0, 50);
        display.print("Pressure:  ");
        display.print(Pressure);
        display.print("hPa");
        display.display();
        Timer_Screen = millis();
    }
}

void screen_stepcounter(){ // for å vise altall skritt på skjerm
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 20);
    display.print("Skritt:");
    display.print(stepSum);
    display.display();
}

void Button()   // avlesning av knapp og for å bytte mellom hva som vises på skjerm
{   
    currentState = digitalRead(BUTTON_PIN); // avlesning av knapp
    if (((millis() - Button_Last) > Button_Read_Speed) && (currentState == LOW))
    {
        if (count == 0) // for å bytte mellom hva som vises på skjerm
        {
            count++;
        }
        else if (count == 1)
        {
            count++;
        }
        else if (count = 2){
            count = 0;
        }
        Button_Last = millis();
    }
    if (count == 0) // for å vis forskjellig informasjon på skjerm
    {
        if (!ubidots.connected())
        {
            screen_offline();
        }
        else
        {
            screen_online();
        }
    }
    else if (count == 1)
    {
        Clock();
    }
    else if (count == 2){
        screen_stepcounter();
    }
}

void stepInit() // Skritteller
{
    x = acc.acceleration.x - xErr;
    y = acc.acceleration.y - yErr;
    z = acc.acceleration.z - zErr;
    vectorMag = sqrt(exp((x - currentx)) + exp((y - currenty)) + exp((z - currentz)));
    if (vectorMag <= currentVectorMag + threshold | vectorMag > maxThreshold)
    {
        sprintf(str, "%d", stepSum);
    }
    else if (vectorMag > currentVectorMag + threshold)
    {
        stepSum++;
    }
    else if (vectorMag > maxThreshold)
    {
    }

    currentx = x;
    currenty = y;
    currentz = z;
    currentVectorMag = vectorMag;
    sprintf(str, "%d", stepSum);
}

void printData()
{
    Serial.print("\t\tAccel X: ");
    Serial.print(acc.acceleration.x);
    Serial.print(" \tY: ");
    Serial.print(acc.acceleration.y);
    Serial.print(" \tZ: ");
    Serial.print(acc.acceleration.z);
    Serial.println(" m/s^2 ");
    Serial.print("\t\tMag X: ");
    Serial.print(mag.magnetic.x);
    Serial.print(" \tY: ");
    Serial.print(mag.magnetic.y);
    Serial.print(" \tZ: ");
    Serial.print(mag.magnetic.z);
    Serial.println(" uT");

    Serial.print("\t\tGyro X: ");
    Serial.print(gyro.gyro.x);
    Serial.print(" \tY: ");
    Serial.print(gyro.gyro.y);
    Serial.print(" \tZ: ");
    Serial.print(gyro.gyro.z);
    Serial.println(" radians/s ");
    Serial.println();

    Serial.print("\t\tTemperature ");
    Serial.print(temp.temperature);
    Serial.println(" deg C");
    if (stepSum > deltaStepSum)
    {
        Serial.print("\t\tSkritt");
        Serial.print(stepSum);
    }
    deltaStepSum = stepSum;
}

void setupBME() // oppkobling av bme280 og skjerm
{
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
    {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;)
            ;
    }
    statusBme = bme.begin(0x76);
    if (!statusBme)
    {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1)
            ;
    }
}
void connectUbidots() // for å koble til ubidots hvis den hgar mistet tilkobling til internett
{
    if (!ubidots.connected()) // skjekker tilkobling
    {
        WiFi.begin(WIFI_SSID, WIFI_PASS); // prøver å koble til internett
        for (int i = 0; i < 1; i++)
        {
            Serial.printf("WIFI Status: %d\n", WiFi.status());
            Serial.print(".");
            WiFi.begin(WIFI_SSID, WIFI_PASS);
            delay(1000);
            if (WiFi.status() != WL_CONNECTED) // ikke tilkoblet
            {
                Serial.println(".");
            }
            if (WiFi.status() == WL_CONNECTED) // tilkoblet
            {
                ubidots.reconnect();
            }
        }
    }
}

void setup()
{
    Serial.begin(115200); // 115200 bps
    setupICM(); // oppkobling av ICM20948
    setupBME(); // oppkobling av BME280 og skjerm
    ubidots.connectToWifi(WIFI_SSID, WIFI_PASS); // kobler til internett
    ubidots.setCallback(callback);  // starter callback 
    ubidots.setup();
    ubidots.reconnect(); // kobler til ubidots
    Timer_Screen = millis();    // starter timer 
    Timer_Publish = millis();
    Timer_Subscribe = millis();
    Button_Last = millis();
    display.clearDisplay(); // Fjerner innhold på skjerm 
    display.setTextSize(1); // for å starte skjermen
    display.setTextColor(WHITE); // for å starte skjermen
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer); // henter klokkeslett og dato
    Clock();    // starter klokke på esp32
    pinMode(BUTTON_PIN, INPUT_PULLUP);  // pin knapp
}

void loop()
{   
    stepInit();
    connectUbidots();
    icm.getEvent(&acc, &gyro, &temp, &mag);
    Button();
    publis();
    subscribe();
    ubidots.loop();
    printData();
}
