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

void obtain() {
  if(sensor.checkForDataReady()) {
    uint16_t rangeStatus = sensor.getRangeStatus();
    int16_t distance = sensor.getDistance(); //uint.
    sensor.clearInterrupt();  // remove
    Serial.print("Center ");
    Serial.print(center[zone]);
    Serial.print(" Distance: ");
    Serial.println(distance);
    //int count = ProcessPeopleCountingData(distance,zone,rangeStatus);
//    if(!count) {
////      Serial.print(count);
//    } else {
//      // Serial.println();
//      // Serial.print("Count: ");
//      // Serial.println(count);
//    }
    zone = zone + 1;
    zone = zone % 2;
    sensor.setROI(ROI_WIDTH,ROI_HEIGHT, center[zone]);
  }
}

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
  setUpTimer(0,obtain,TIMING_BUDGET+4);
  startTimer(0);
}


int ProcessPeopleCountingData(int16_t Distance, uint8_t zone, uint8_t RangeStatus) {
  static int PathTrack[] = {0,0,0,0};
  static int PathTrackFillingSize = 1; // init this to 1 as we start from state where nobody is any of the zones
  static int LeftPreviousStatus = NOBODY;
  static int RightPreviousStatus = NOBODY;
  static int PeopleCount = 0;
  static uint16_t Distances[2][DISTANCES_ARRAY_SIZE];
  static uint8_t DistancesTableSize[2] = {0,0};

  uint16_t MinDistance;
  uint8_t i;

  int CurrentZoneStatus = NOBODY;
  int AllZonesCurrentStatus = 0;
  int AnEventHasOccured = 0;
  
  /*Adds inputted distance to Distances matrix (10 samples (DISTANCES_ARRAY_SIZE as defined at top)*/
  if (DistancesTableSize[zone] < DISTANCES_ARRAY_SIZE) {
    Distances[zone][DistancesTableSize[zone]] = Distance;
    DistancesTableSize[zone] ++;
  }
  else { // otherewise shift left & refresh last entry to most recent distance
    for (i=1; i<DISTANCES_ARRAY_SIZE; i++)
      Distances[zone][i-1] = Distances[zone][i];
    Distances[zone][DISTANCES_ARRAY_SIZE-1] = Distance;
  }
  
  // Use minDistance from "10" (DISTANCES_ARRAY_SIZE) sample for threshold pass determination
  MinDistance = Distances[zone][0];
  if (DistancesTableSize[zone] >= 2) {
    for (i=1; i<DistancesTableSize[zone]; i++) {
      if (Distances[zone][i] < MinDistance)
        MinDistance = Distances[zone][i];
    }
  }
  
  if (MinDistance < DIST_THRESHOLD) {
    // Someone is in !
    CurrentZoneStatus = SOMEONE;
  }
  
  // left zone
  if (zone == LEFT) {
    if (CurrentZoneStatus != LeftPreviousStatus) {
      // event in left zone has occured
      AnEventHasOccured = 1;
      
      if (CurrentZoneStatus == SOMEONE) {
        AllZonesCurrentStatus += 1;
      }
      // need to check right zone as well ...
      if (RightPreviousStatus == SOMEONE) {
        // event in left zone has occured
        AllZonesCurrentStatus += 2;
      }
      // remember for next time
      LeftPreviousStatus = CurrentZoneStatus;
    }
  }
  // right zone
  else {
    
    if (CurrentZoneStatus != RightPreviousStatus) {
      // event in right zone has occured
      AnEventHasOccured = 1;
      if (CurrentZoneStatus == SOMEONE) {
        AllZonesCurrentStatus += 2;
      }
      // need to left right zone as well ...
      if (LeftPreviousStatus == SOMEONE) {
        // event in right zone has occured
        AllZonesCurrentStatus += 1;
      }
      // remember for next time
      RightPreviousStatus = CurrentZoneStatus;
    }
  }
  // if an event has occured
  if (AnEventHasOccured) {
    if (PathTrackFillingSize < 4) {
      PathTrackFillingSize ++;
    }
    
    // if nobody anywhere lets check if an exit or entry has happened
    if ((LeftPreviousStatus == NOBODY) && (RightPreviousStatus == NOBODY)) {
      
      // check exit or entry only if PathTrackFillingSize is 4 (for example 0 1 3 2) and last event is 0 (nobobdy anywhere)
      if (PathTrackFillingSize == 4) {
        // check exit or entry. no need to check PathTrack[0] == 0 , it is always the case
        
        if ((PathTrack[1] == 1)  && (PathTrack[2] == 3) && (PathTrack[3] == 2)) {
          // This an entry
          PeopleCount = PeopleCount + 1;
          // reset the table filling size in case an entry or exit just found
          DistancesTableSize[0] = 0;
          DistancesTableSize[1] = 0;
          Serial.print("Walk In, People Count=");
          Serial.println(PeopleCount);
        } else if ((PathTrack[1] == 2)  && (PathTrack[2] == 3) && (PathTrack[3] == 1)) {
          // This an exit
          PeopleCount = PeopleCount - 1;
          // reset the table filling size in case an entry or exit just found
          DistancesTableSize[0] = 0;
          DistancesTableSize[1] = 0;
          Serial.print("Walk Out, People Count="); 
          Serial.println(PeopleCount);
        } else {
          // reset the table filling size also in case of unexpected path
          DistancesTableSize[0] = 0;
          DistancesTableSize[1] = 0;
          Serial.println("Wrong path");
        }
      }
      
      PathTrackFillingSize = 1;
    }
    else {
      // update PathTrack
      // example of PathTrack update
      // 0
      // 0 1
      // 0 1 3
      // 0 1 3 1
      // 0 1 3 3
      // 0 1 3 2 ==> if next is 0 : check if exit
      PathTrack[PathTrackFillingSize-1] = AllZonesCurrentStatus;
    }
  }
  return PeopleCount;
}
