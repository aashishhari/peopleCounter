#include "hal.h"
#include "HX711.h"
#include "soc/rtc.h"
#include "hal.h"

#include <Wire.h>
#include <SparkFun_VL53L1X.h>
//#include <WiFi.h>

//wifi globals
//const char* ssid = "SealeoG";
//const char* password =  "rhshdh2000";
// 
//const uint16_t port = 8090;
//const char * host = "192.168.1.8";

//ToF Globals
static const int NOBODY = 0;
static const int SOMEONE = 1;
static const int LEFT = 0;
static const int RIGHT = 1;
static int center[2] = {167, 231}; // andrea's suggested value also try
int zone = 0;
// Specific to our Profile
#define DISTANCES_ARRAY_SIZE                         10   // nb of samples
#define DIST_THRESHOLD                               1600  // mm
#define ROWS_OF_SPADS                                4 // 8x16 SPADs ROI
#define TIMING_BUDGET                                33  // was 20 ms, I found 33 ms has better succes rate with lower reflectance target
#define ROI_WIDTH                                    4
#define ROI_HEIGHT                                   4
SFEVL53L1X sensor(Wire);
int ProcessPeopleCountingData(int16_t Distance, uint8_t zone, uint8_t RangeStatus);
int count = 0;t
int oldCount = 0;
/** done **/

//load sensor globals
HX711 scale;
int peepcount = 0;
float threshold = 1000; // kg
byte trials = 1;

//final code globals
int people = 0;
int lastactive = 0;
int lastbothactive = 0;
int timeBetweenActivation = 3000;
int timeBetweenBothTrigger = 500;
 int loadtime = 0;
 int toftime = 0;
volatile int counter = 0;

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  //1. setup load sensor
  //Server setup
//  WiFi.begin(ssid, password);
//  while (WiFi.status() != WL_CONNECTED) {
//    delay(500);
//    Serial.println("...");
//  }
// 
//  Serial.print("WiFi connected with IP: ");
//  Serial.println(WiFi.localIP());

  //load sensor setup
  rtc_clk_cpu_freq_set(RTC_CPU_FREQ_80M);

  Serial.println("Initializing the scale");
  // parameter "gain" is ommited; the default value 128 is used by the library
  // HX711.DOUT  - pin #A1
  // HX711.PD_SCK - pin #A0
  scale.begin(26, 25);
//
//  // uncomment if you want to calibrate the bowl
//  scale. set_scale();
//  scale.tare();
//  Serial.println("Put known weight on ");
//  //displayString("Calibrate", 64, 10);
//  delay(2500);
//  Serial.print(scale.get_units(10));
//  Serial.print(" Divide this value to the weight and insert it in the scale.set_scale() statement");
////  displayFloat(scale.get_units(10), 64, 15);
//  while(1==1);
//
  Serial.println("Before setting up the scale:");
  Serial.print("read: \t\t");
  Serial.println(scale.read());     // print a raw reading from the ADC

  Serial.print("read average: \t\t");
  Serial.println(scale.read_average(20));   // print the average of 20 readings from the ADC

  Serial.print("get value: \t\t");
  Serial.println(scale.get_value(5));   // print the average of 5 readings from the ADC minus the tare weight (not set yet)

  Serial.print("get units: \t\t");
  Serial.println(scale.get_units(5), 1);  // print the average of 5 readings from the ADC minus tare weight (not set) divided
  // by the SCALE parameter (not set yet)

  scale.set_scale(50);    // this value is obtained by calibrating the scale with known weights; see the README for details
  scale.tare();               // reset the scale to 0

  Serial.println("After setting up the scale:");

  Serial.print("read: \t\t");
  Serial.println(scale.read());                 // print a raw reading from the ADC

  Serial.print("read average: \t\t");
  Serial.println(scale.read_average(20));       // print the average of 20 readings from the ADC

  Serial.print("get value: \t\t");
  Serial.println(scale.get_value(5));   // print the average of 5 readings from the ADC minus the tare weight, set with tare()

  Serial.print("get units: \t\t");
  Serial.println(scale.get_units(5), 1);        // print the average of 5 readings from the ADC minus tare weight, divided
  // by the SCALE parameter set with set_scale

  Serial.println("Readings:");
//  client.publish("esp/test", "Hello from ESP32");
//  setUpTimer(0, count_person, 1000000);
//  startTimer(0);
  //load_sensor_setup();
  

  /* SETUP TOF SENSOR */
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C 0x52
  // Failure Mode 1
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

void flip() {
  counter++;
  if(counter > 5000) {
    counter = 0;
  }
//  if(3000 < counter && counter < 3500) g{
//    loadtime = milddwdadadsgfsgfdfgdsssssssdflis();
//  }
  if(4000 < counter && counter < 4500) {
    toftime = millis();
  }
}

void loop() {
  //1. perform one iteration of the load sensor: returns the last time it was pressed down
//  WiFiClient client;
  float reading = scale.get_units(1);
  //Serial.println(reading);
  if(reading > threshold){ // maybe include a boolean check to see if the TOF is picking up something also
    //peepcount = peepcount + 1;
    loadtime = millis();
    //Serial.println(peepcount);
    //delay(100);
    
//    if (!client.connect(host, port)) {
// 
//        Serial.println("Connection to host failed");
// 
//        delay(1000);
//        return;
//    }
// 
//    Serial.println("Connected to server successful!");
// 
//    client.print(peepcount);
// 
//    Serial.println("Disconnecting...");
//    client.stop();
// 
//    delay(1000);
  }
  //2. perform one iteration of the tof: returns the last time it was triggered
  if(sensor.checkForDataReady()) {
    uint16_t rangeStatus = sensor.getRangeStatus();
    int16_t distance = sensor.getDistance(); //uint.
    sensor.clearInterrupt();
    count = ProcessPeopleCountingData(distance,zone,rangeStatus);
    zone = zone + 1;
    zone = zone % 2;
    sensor.setROI(ROI_WIDTH,ROI_HEIGHT, center[zone]);
  }
  if(oldCount != count) {
    toftime = millis();
  }
  oldCount = count;
  //toftime = millis();
  //delete count from the code and instead return millis during the trigger
  //int loadtime = 0; //these two variables get initialized by the above two lines
  //int toftime = 0;  //use millis to get these two times

  //do some code that determines if a person walked in or out
  if (loadtime == 0 && toftime == 0 || (millis() - lastbothactive < timeBetweenBothTrigger)) {
    
  }
  else if (loadtime != 0 && toftime == 0) {
    lastactive = loadtime;
  }
  else if (loadtime == 0 && toftime != 0) {
    lastactive = toftime;
  }
  else {
    int bothactive = millis();
    Serial.println(bothactive - lastactive);
    if (bothactive - lastactive < timeBetweenActivation) {
      
      if (toftime - loadtime > 1500) {
        people++; //implement wifi
        Serial.println(people);
        Serial.println(9237594325724895);
      }
      else {
        if(people > 0) {
          people--;
      }//implement wifi
        Serial.println(people);
        Serial.println(9237594325724895);
      }
      lastbothactive = bothactive;
      loadtime = 0;
      toftime = 0;
    }
    else {
      lastactive = bothactive;
      if(toftime > loadtime) {
        loadtime = 0;
      }
      else {
        toftime = 0;
      }
    }
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
