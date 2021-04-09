#include "hal.h"

int people = 0;
int lastactive = 0;
int lastbothactive = 0;
int timeBetweenActivation = 1000;
int timeBetweenBothTrigger = 500;
 int loadtime = 0;
 int toftime = 0;
volatile int counter = 0;

void setup() {
  // put your setup code here, to run once:
  //1. setup load sensor
  //2. setup tof sensor
  setUpTimer(1, flip, 1000);
  startTimer(1);
}

void flip() {
  counter++;
  if(counter > 10000) {
    counter = 0;
  }
  if(3000 < counter && counter < 3500) {
    loadtime = counter;
  }
  else if(4000 < counter && counter < 4500) {
    toftime = counter;
  }
}

void loop() {
  //1. perform one iteration of the load sensor: returns the last time it was pressed down
  //2. perform one iteration of the tof: returns the last time it was triggered
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
    if (bothactive - lastactive < timeBetweenActivation) {
      if (toftime > loadtime) {
        people++; //implement wifi
        Serial.println(people);
      }
      else {
        people--; //implement wifi
        Serial.println(people);
      }
      lastbothactive = bothactive;
      loadtime = 0;
      toftime = 0;
    }
    else {
      lastactive = 0;
      loadtime = 0;
      toftime = 0;
    }
  }
}
