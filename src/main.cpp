// Bibliotek
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// ICM-20948
Adafruit_ICM20948 icm;
sensors_event_t acc;
sensors_event_t temp;
sensors_event_t mag;
sensors_event_t gyro;

// - Deklarasjon -
static unsigned long stepSum = 0;
int deltaStepSum = 0;

// Akselerasjon
float x, y, z;
static float currentx = 0;
static float currenty = 0;
static float currentz = 0;
static float currentVectorMag = 0;

const float xErr = 0.80; // Sensorkorigeringsverdier.
const float yErr = 0.50;
const float zErr = 10.22;
const float threshold = 0.05; // Grense for stegakselerasjon
float vectorMag;

// Diverse
char str[50];
byte buff[10];

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

  Serial.println(stepSum);
}



void stepInit()
{
  x = acc.acceleration.x - xErr;
  y = acc.acceleration.y - yErr;
  z = acc.acceleration.z - zErr;
  vectorMag = sqrt(exp((x - currentx)) + exp((y - currenty)) + exp((z - currentz)));
  if (vectorMag <= currentVectorMag + threshold)
  {
    sprintf(str, "%d", stepSum);
  }
  else if (vectorMag > currentVectorMag + threshold)
  {
    stepSum++;
  }
  currentx = x;
  currenty = y;
  currentz = z;
  currentVectorMag = vectorMag;
  sprintf(str, "%d", stepSum);
}
bool stepAssert() // Debugging throw error if wiring wrong
{
}
void setup()
{
  Serial.begin(115200);
  bool status = icm.begin_I2C(0x68);
  if (!status)
  {
    Serial.println("Could not find a valid ICM sensor, check wiring!");
    while (1)
      ;
  }
  Serial.println("ICM20948 Found!");
  Serial.println("OK");

  icm.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
  icm.setGyroRateDivisor(255);
  icm.setMagDataRate(AK09916_MAG_DATARATE_10_HZ);
  icm.setAccelRange(ICM20948_ACCEL_RANGE_16_G);
  icm.setAccelRateDivisor(4095);
}

void loop()
{
  icm.getEvent(&acc, &gyro, &temp, &mag);
  stepInit();
  
  if(stepSum>deltaStepSum){
    printData();
  }
  deltaStepSum = stepSum;
}