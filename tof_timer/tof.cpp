#include <Wire.h>
#include <SparkFun_VL53L1X.h>

#define NOBODY                    0
#define SOMEONE                   1
#define LEFT                      0
#define RIGHT                     1

#define DISTANCES_ARRAY_SIZE                         10   // nb of samples
#define DIST_THRESHOLD                               1600  // mm
#define ROWS_OF_SPADS                                4 // 8x16 SPADs ROI
#define TIMING_BUDGET                                33  // was 20 ms, I found 33 ms has better succes rate with lower reflectance target
#define ROI_WIDTH                                    4
#define ROI_HEIGHT                                   4

static int center[2] = {167, 231}; // FRONT_ZONE_CENTER, BACK_ZONE_CENTER
int zone = 0;

SFEVL53L1X sensor(Wire);

void tofInit() {
  Wire.begin();
  Wire.setClock(400000); // Required I2C frequency
  
  if (sensor.begin()) // init() is deprecated version
  {
    Serial.println("Error Connecting to Sensor");
    while(1);
  }
  Serial.println("Success connecting to sensor");

  sensor.setDistanceModeLong(); // modify this mode.
  sensor.setTimingBudgetInMs(TIMING_BUDGET);
  sensor.setIntermeasurementPeriod(TIMING_BUDGET+4); // The minimum inter-measurement period must be longer than the timing budget + 4 ms - UM2356 (perhaps we can get away with 1 we'll see)
  sensor.setROI(ROI_WIDTH,ROI_HEIGHT,center[zone]);
  sensor.startRanging();
  Serial.println("Starting...");
}

void range() {
  
}
