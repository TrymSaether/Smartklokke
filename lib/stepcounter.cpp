/* #include "Stepcounter.h"

ICM_20948 sensor;

#define READ 6

static unsigned long stepSum = 0;
static int currentxx = 0;
static int currentyy = 0;
static int currentzz = 0;
char str[50];
byte buff[READ];
int xx, yy, zz;
int vector;

void stepInit(void)
{
    xx = sensor.accX();
    yy = sensor.accY();
    zz = sensor.accZ();
    vector = sqrt(exp((xx - currentxx)) + exp((yy - currentyy)) + exp((zz - currentzz)));
    if (vector < 50)
    {
        sprintf(str, "%d", stepSum);
        return;
    }
    else if (vector > 50)
    {
        stepSum++;
    }
    currentxx = xx;
    currentyy = yy;
    currentzz = zz;
    sprintf(str, "%d", stepSum);
}
bool stepAssert() // Debugging throw error if wiring wrong
{

}
*/