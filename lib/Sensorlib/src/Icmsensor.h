#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

void printData(); // Printer aks, gyro, magn og temp data.
void stepInit();
void setupICM();
void velocityICM();