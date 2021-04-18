#include "hal.h"
#include "HX711.h"
#include "soc/rtc.h"
#include "hal.h"

#include <Wire.h>
#include <SparkFun_VL53L1X.h>
#include <WiFi.h>

/** WIFI Globals **/
const char* ssid = "NETGEAR92";
const char* password =  "youngtrumpet129";
const uint16_t port = 8090;
const char * host = "192.168.1.16";
WiFiClient client;
/** done **/

/** TOF Globals **/
static const int NOBODY = 0;
static const int SOMEONE = 1;
static const int LEFT = 0;
static const int RIGHT = 1;
static int center[2] = {167, 231}; // andrea's suggested value also try
int zone = 0;

#define DIST_THRESHOLD                               1600  // mm
#define ROWS_OF_SPADS                                4 // 8x16 SPADs ROI
#define TIMING_BUDGET                                33  // was 20 ms, I found 33 ms has better succes rate with lower reflectance target
#define ROI_WIDTH                                    4
#define ROI_HEIGHT                                   4
SFEVL53L1X sensor(Wire);
int ProcessPeopleCountingData(int16_t Distance, uint8_t zone);
int count = 0;
int oldCount = 0;
/** done **/

/** Load Sensor Globals **/
HX711 scale;
int peepcount = 0;
float threshold = 1000; // kg
byte trials = 1;
/** done **/

/** FINAL CODE Globals **/
int oldwificount = 0;
int people = 0;
int oldpeoplecount = 0;
int lastactive = 0;
int lastbothactive = 0;
int timeBetweenActivation = 3000;
int timeBetweenBothTrigger = 500;
 int loadtime = 0;
 int toftime = 0;
volatile int counter = 0;
/** done **/

void setup() {
  Serial.begin(9600);
  
  /* SERVER Setup */
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("...");
  }
  Serial.print("WiFi connected with IP: ");
  Serial.println(WiFi.localIP());
  /* --- */

  /* 1. Setup Load Sensor */
  /* Notes:
  parameter "gain" is ommited; the default value 128 is used by the library 
  HX711.DOUT  - pin #A1
  HX711.PD_SCK - pin #A0
  */
  rtc_clk_cpu_freq_set(RTC_CPU_FREQ_80M);
  Serial.println("Initializing the scale");
  scale.begin(26, 25);

  /* uncomment if you want to calibrate the bowl
  scale. set_scale();
  scale.tare();
  Serial.println("Put known weight on ");
  //displayString("Calibrate", 64, 10);
  delay(2500);
  Serial.print(scale.get_units(10));
  Serial.print(" Divide this value to the weight and insert it in the scale.set_scale() statement");
  //  displayFloat(scale.get_units(10), 64, 15);
  while(1==1);
  */

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
  /* --- */

//  client.publish("esp/test", "Hello from ESP32");
//  setUpTimer(0, count_person, 1000000);
//  startTimer(0);
  //load_sensor_setup();

  //setUpTimer(1, setupwifi, 50000); 
  //startTimer(1);1
  

  /* SETUP TOF SENSOR */
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C 0x52
  if (sensor.begin()) { // init() is deprecated()
    Serial.println("Error Connecting to Sensor");
    while(1); // oh maybe I can fix this...do a loop type thing.
  }
  Serial.println("Success connecting to sensor");
  sensor.setDistanceModeLong(); // modify this mode.
  sensor.setTimingBudgetInMs(TIMING_BUDGET);
  sensor.setIntermeasurementPeriod(TIMING_BUDGET+4); // The minimum inter-measurement period must be longer than the timing budget + 4 ms - UM2356 (perhaps we can get away with 1 we'll see)
  sensor.setROI(ROI_WIDTH,ROI_HEIGHT,center[zone]);
  sensor.startRanging();
  Serial.println("Starting...");
  /* --- */
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

//float reading = 0;
float prevReading = 0;
void loop() { // ok need to walk through this code after peopleCount;
  //1. perform one iteration of the load sensor: returns the last time it was pressed down
  // prevReading = reading;
  float reading = scale.get_units(1);
  Serial.println(reading);
  if(reading > threshold){ // maybe include a boolean check to see if the TOF is picking up something also
    //peepcount = peepcount + 1;
    loadtime = millis(); //Serial.println(peepcount); //delay(100);
  }

  /* Alternate Approach (will need threshold modification)
  if(abs(reading - prevReading) > threshold ) {
    loadtime = millis();
  }
  */
  
  //2. perform one iteration of the tof: returns the last time it was triggered
  if(sensor.checkForDataReady()) {
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

  //do some code that determines if a person walked in or out
  if (loadtime == 0 && toftime == 0 || (millis() - lastbothactive < timeBetweenBothTrigger)) {
    // do nothing
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
        setupwifi();
      }
      else {
        if(people > 0) {
          people--;
      }//implement wifi
        Serial.println(people);
        setupwifi();
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

// run on separate core.
void setupwifi() {
  if(oldpeoplecount != people && millis()-oldwificount > 500) {
    oldwificount = millis();
    oldpeoplecount = people;
      if (!client.connect(host, port)) {
 
        Serial.println("Connection to host failed");
 
        delay(1000);
        return;
    }
 
    Serial.println("Connected to server successful!");
 
    client.print(people);
 
    Serial.println("Disconnecting...");
    client.stop();
 
    //delay(1000); 
  }
}

int ProcessPeopleCountingData(int16_t Distance, uint8_t zone) {
  static int PathTrack[] = {0,0,0,0};
  static int PathTrackFillingSize = 1; // init this to 1 as we start from state where nobody is any of the zones
  static int LeftPreviousStatus = NOBODY;
  static int RightPreviousStatus = NOBODY;
  static int PeopleCount = 0; // remember this is GLOBAL!

  int CurrentZoneStatus = NOBODY;
  int AllZonesCurrentStatus = 0;
  int AnEventHasOccured = 0;

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
        // event in right zone has occured
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
      PathTrackFillingSize++;
    }
    
    // if nobody anywhere lets check if an exit or entry has happened
    if ((LeftPreviousStatus == NOBODY) && (RightPreviousStatus == NOBODY)) {
      // check exit or entry only if PathTrackFillingSize is 4 (for example 0 1 3 2) and last event is 0 (nobobdy anywhere)
      if (PathTrackFillingSize == 4) {
        // check exit or entry. no need to check PathTrack[0] == 0 , it is always the case NOBODY
        if ((PathTrack[1] == 1)  && (PathTrack[2] == 3) && (PathTrack[3] == 2)) {
          // This an entry
          PeopleCount = PeopleCount + 1;
          // reset the table filling size in case an entry or exit just found
          Serial.print("Walk In, People Count=");
          Serial.println(PeopleCount);
        } else if ((PathTrack[1] == 2)  && (PathTrack[2] == 3) && (PathTrack[3] == 1)) {
          // This an exit
          PeopleCount = PeopleCount - 1;
          // reset the table filling size in case an entry or exit just found
          Serial.print("Walk Out, People Count="); 
          Serial.println(PeopleCount);
        } else {
          // reset the table filling size also in case of unexpected path
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
