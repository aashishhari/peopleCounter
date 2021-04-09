int people = 0;
int lastactive = 0;
int lastbothactive = 0;
int timeBetweenActivation = 1000;
int timeBetweenBothTrigger = 500;
void setup() {
  // put your setup code here, to run once:
  //1. setup load sensor
  //2. setup tof sensor
}

void loop() {
  //1. perform one iteration of the load sensor: returns the last time it was pressed down
  //2. perform one iteration of the tof: returns the last time it was triggered
  //delete count from the code and instead return millis during the trigger
  int loadtime = 0; //these two variables get initialized by the above two lines
  int toftime = 0;  //use millis to get these two times
  
  //do some code that determines if a person walked in or out
  if (loadtime == 0 && toftime == 0 || (millis()-lastbothactive < timeBetweenBothTrigger)) {
    continue;
  }
  else if (loadtime != 0 && toftime == 0) {
    lastactive = loadtime;
  }
  else if(loadtime == 0 && toftime != 0) {
    lastactive = toftime;
  }
  else {
    int bothactive = millis();
    if (bothactive-lastactive < timeBetweenActivation) {
      if(toftime>loadtime) {
        people++; //implement wifi 
      }
      else {
        people--; //implement wifi
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
