#include <Wire.h>
#include <SparkFun_VL53L1X.h>

#define SHUTDOWN_PIN 2 //idk
#define INTERRUPT_PIN 3 //idk


const int threshold_percentage = 80;

char peopleCounterArray[50];

static const int NOBODY = 0;
static const int SOMEONE = 1;
static const int LEFT = 0;
static const int RIGHT = 1;

static int center[2] = {167, 231}; // andrea's suggested value also try
int zone = 0;

// Specific to our Profile
#define PROFILE_STRING                               "DOOR_JAM_2400"
#define DISTANCES_ARRAY_SIZE                         10   // nb of samples
#define MAX_DISTANCE                                 2400 // mm, 2000
#define MIN_DISTANCE                                 0   // mm
#define DIST_THRESHOLD                               850  // mm, 1600
#define ROWS_OF_SPADS                                8 // 8x16 SPADs ROI
#define TIMING_BUDGET                                33  // was 20 ms, I found 33 ms has better succes rate with lower reflectance target, ignor eusing 50
#define DISTANCE_MODE                                DISTANCE_MODE_LONG

SFEVL53L1X sensor(Wire);

//void zonesCalibration();
int ProcessPeopleCountingData(int16_t Distance, uint8_t zone, uint8_t RangeStatus);
int oldCount = 0;
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  // Failure Mode 1
  if (sensor.begin()) // init() is deprecated version
  {
    Serial.println("Error Connecting to Sensor");
    while(1);
  }
  Serial.println("Success connecting to sensor");
  
  // Minimum timing budget is 20ms for short distance mode and 33ms for medium & long distance modes ==> Check datasheet for more details.
  sensor.setDistanceModeLong(); // discrete set of supported measurement frequencies for ULD [20 33 50 100] [15, 20, 33, 50, 100, 200, 500] , don't think 15 is actually allowed, 66 Hz max ranging, 33 ms min for our application
  sensor.setTimingBudgetInMs(33); // timing budget for measurement

  // Start continuous readings at a rate of one measurement every 50 ms (the inter-measurement period). 
  // This period should be at least as long as the timing budget.
  sensor.setIntermeasurementPeriod(34); // time between measurements. (Delay between Ranging operations)
  sensor.setROI(4,4,center[zone]);

  // sensor zone calibration -- ok this is how we get the zonedata, to implement later (add next line there as well)
  sensor.startRanging();
  Serial.println("Starting...");
}

// use millis() instead of delay()https://dzone.com/articles/arduino-using-millis-instead-of-delay

void loop() {
  // put your main code here, to run repeatedly:
  if(sensor.checkForDataReady()) {
    //sensor.setDistanceThreshold(100,300,3); 
    //setDist(dev,100,300,0,1): below 100 ||| setDist(dev,100,300,1,1): above 300 ||| setDist(dev,100,300,2,1): out-of-window ||| setDist(dev,100,300,3,1): in window
    uint16_t rangeStatus = sensor.getRangeStatus();
    int16_t distance = sensor.getDistance(); //uint.
    sensor.clearInterrupt();
//    Serial.print("Center ");
//    Serial.print(center[zone]);
//    Serial.print(" Distance: ");
//    Serial.println(distance);
    int count = ProcessPeopleCountingData(distance,zone,rangeStatus);
    if(!count) {
      Serial.print(count);
    } else {
      Serial.println();
      Serial.print("Count: ");
      Serial.println(count);
      oldCount = count;
    }
    zone = zone + 1;
    zone = zone % 2;
    sensor.setROI(4,4, center[zone]);
    
  }
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
  
  // Add just picked distance to the table of the corresponding zone
  if (DistancesTableSize[zone] < DISTANCES_ARRAY_SIZE) {
    Distances[zone][DistancesTableSize[zone]] = Distance;
    DistancesTableSize[zone] ++;
  }
  else {
    for (i=1; i<DISTANCES_ARRAY_SIZE; i++)
      Distances[zone][i-1] = Distances[zone][i];
    Distances[zone][DISTANCES_ARRAY_SIZE-1] = Distance;
  }
  
  // pick up the min distance
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
      
      // event in left zone has occured
      AnEventHasOccured = 1;
      if (CurrentZoneStatus == SOMEONE) {
        AllZonesCurrentStatus += 2;
      }
      // need to left right zone as well ...
      if (LeftPreviousStatus == SOMEONE) {
        // event in left zone has occured
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

// 231 167
//int ProcessPeopleCountingData(int16_t Distance, uint8_t zone, uint8_t RangeStatus) {
//  //
//}

///* TODO
// *  p9 Calibration Details (offset, crosstalk (coverglass thing only maybe?) -- idk really how to do this part: https://www.st.com/content/ccc/resource/technical/document/user_manual/group1/fc/c3/0b/8c/0c/da/4c/8d/DM00600212/files/DM00600212.pdf/jcr:content/translations/en.DM00600212.pdf
// */
// */
