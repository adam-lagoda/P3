//importing libraries
#include <BasicStepperDriver.h>
#include <IRsensor.h>
#include <Servo.h>
#include <LiquidCrystal.h>
#include <Path.h>
#include <SoftwareSerial.h>
#include "VoiceRecognitionV3.h"

//pathdata--------------------------------------------------------------
/*MAX_HELPNODES must be 5 - CHANGE IN Path.h!*/
byte pathsnumber = 15;
Coord ab[] = {{18, 36}, {16, 36}, {16, 23}, {11, 23}, {11, 36}};
Coord ae[] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}};
Coord ac[] = {{18, 13}, {8, 13}, {8, 8}, {19, 8}, {0, 0}};
Coord ad[] = {{25, 37}, {25, 23}, {30, 23}, {30, 37}, {0, 0}};
Coord af[] = {{25, 37}, {25, 23}, {33, 23}, {0, 0}, {0, 0}};
Coord bc[] = {{8, 36}, {8, 8}, {19, 8}, {0, 0}, {0, 0}};
Coord be[] = {{11, 36}, {11, 23}, {16, 23}, {16, 37}, {0, 0}};
Coord bd[] = {{11, 36}, {11, 23}, {30, 23}, {30, 37}, {0, 0}};
Coord bf[] = {{11, 36}, {11, 23}, {33, 23}, {0, 0}, {0, 0}};
Coord cd[] = {{19, 8}, {8, 8}, {8, 23}, {30, 23}, {30, 37}};
Coord ce[] = {{19, 8}, {8, 8}, {8, 23}, {16, 23}, {16, 37}};
Coord cf[] = {{19, 8}, {8, 8}, {8, 13}, {33, 13}, {0, 0}};
Coord de[] = {{30, 37}, {30, 23}, {25, 23}, {25, 37}, {0, 0}};
Coord df[] = {{33, 37}, {0, 0}, {0, 0}, {0, 0}, {0, 0}};
Coord ef[] = {{25, 37}, {25, 23}, {33, 23}, {0, 0}, {0, 0}};
Path paths[] = {Path({"a", 18, 37}, {"b", 5, 36}, ab), Path({"a", 18, 37}, {"e", 22, 37}, ae), Path({"a", 18, 37}, {"c", 19, 5}, ac), Path({"a", 18, 37}, {"d", 32, 37}, ad), Path({"a", 18, 37}, {"f", 33, 3}, af), Path({"b", 5, 36}, {"c", 19, 5}, bc), Path({"b", 5, 36}, {"e", 22, 37}, be), Path({"b", 5, 36}, {"d", 32, 37}, bd), Path({"b", 5, 36}, {"f", 33, 3}, bf), Path({"c", 19, 5}, {"d", 32, 37}, cd), Path({"c", 19, 5}, {"e", 22, 37}, ce), Path({"c", 19, 5}, {"f", 33, 3}, cf), Path({"d", 32, 37}, {"e", 22, 37}, de), Path({"d", 32, 37}, {"f", 33, 3}, df), Path({"e", 22, 37}, {"f", 33, 3}, ef)};


//Voice recognition-----------------------------------------------------
VR myVR(50, 52); //iniitalize board on pins 50,52
uint8_t records[7]; // save record
uint8_t buf[64];
//define signatures
#define station (0) 
#define bedroom (1)
#define livingroom (2)
#define bathroom (3)
#define kitchen (4)
#define door (5)
//define pins for pushtotalk and decline
const int pushToTalkPin = 3;
const int declinePin = 5;

//function to start voicecontrol board
void startup(){
  if (myVR.clear() == 0) {
    Serial.println("Recognizer cleared.");
  } else {
    Serial.println("Not find VoiceRecognitionModule.");
    Serial.println("Please check connection and restart Arduino.");
    while (1);
  }
  if (myVR.load((uint8_t)station) >= 0) {
   Serial.println("Station");
  }
  if (myVR.load((uint8_t)bedroom) >= 0) {
    Serial.println("bedroom");
  }
  if (myVR.load((uint8_t)livingroom) >= 0) {
    Serial.println("livingroom");
  }
  if (myVR.load((uint8_t)bathroom) >= 0) {
   Serial.println("bathroom");
  }
  if (myVR.load((uint8_t)kitchen) >= 0) {
    Serial.println("kitchen");
  }
  if (myVR.load((uint8_t)door) >= 0) {
    Serial.println("door");
  }
}

//function to return a char based on signature/voice command
char printVR(){
  int ret;
  ret = myVR.recognize(buf, 50);
  if (ret > 0) {
    switch (buf[1]) {
      case station:
        return 'a';
        break;
          
      case kitchen:
        return 'b';
        break;

      case livingroom:
        return 'c';
        break;

      case bathroom:
        return 'd';
        break;

      case bedroom:
        return 'e';
        break;
        
      case door:
        return 'f';
        break;
        
      default:
        return 'g';
        break;
    }
  }
  else{
    return '!';
  }
}

String newposition; //next position the robot must move to
String currentposition = "a"; //waypoint where the robot starts

//Driving---------------------------------------------------------------
#define MOTORSTEP 200 //steps per revolution
#define RPM 50 //speed
#define MICROSTEPS 1 //microsteps disabled
#define DIR1 8 //direction pin for left motor 
#define DIR2 6 //direction pin for right motor
#define STEPPIN 9 //step signal pin
#define STEPLEFT 106 //steps to turn left
#define STEPRIGHT 125 //steps to turn right
#define GRIDSIZE 5 //grid cells are 5x5 cm
#define GRIDSTEP 58 //steps pr. grid 58 steps is roughly 5 cm
#define FORWARDGRID 3 //grid cells to move forward
#define FORWARDDIST FORWARDGRID*GRIDSTEP //steps
byte hdg = 1; //north=1, west=2, south=3, east=4
BasicStepperDriver stepper(MOTORSTEP, -1, STEPPIN);

//Obstacle detection----------------------------------------------------
#define SERVOPIN 4 //pin for the servo
#define MINDISTANCE 13 //distance for obstacle avoidance
#define COLLISIONDIST 8 //distance where the collision detection reacts
#define ROBOTGRID 5 //how many grid cells the robot takes in length
#define ROBOTLENGTH ROBOTGRID*GRIDSTEP //length of robot in steps
Servo irservo; //servo object
IRsensor front = IRsensor(A8,irservo); //obstacle detetction
IRsensor collision = IRsensor(A9); //colission detection

//Battery level indicator/display---------------------------------------
int battPin = A5; //connected to the voltage divider
int voltReading; //raw input from voltage divider
int output; //mV from voltage divider
int volts; //battery percentage
byte battery[8] = {//battery symbol
  0b00000,
  0b00100,
  0b01110,
  0b01110,
  0b01110,
  0b01110,
  0b01110,
  0b00000
};
LiquidCrystal lcd(37,39,41,43,45,47); //start lcd

//Emergency stop-------------------------------------
const byte emergencyStopPin = 2; //pin for emergency stop
void emergencyStop(){
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > 200){
    lcd.setCursor(12,1); //print to lcd
    lcd.print("Stop");
    //wait for button to be released and pressed again
    while(digitalRead(emergencyStopPin)==LOW){
    }
    while(digitalRead(emergencyStopPin)==HIGH){
    }
    lcd.setCursor(12,1); //clear the "stop" from the lcd
    lcd.print("     ");
  }
  last_interrupt_time = interrupt_time;
}

void setup(){
  Serial.begin(9600); //start serial
  irservo.attach(SERVOPIN); //attach servo to pin
  stepper.begin(RPM, MICROSTEPS); //initialize steppers
  lcd.begin(16,2); //start lcd
  lcd.createChar(0, battery); //define the battery symbol
  pinMode(battPin, INPUT); //define battpin as input
  pinMode(emergencyStopPin, INPUT_PULLUP); //define the emergency stop as input and interrupt
  attachInterrupt(digitalPinToInterrupt(emergencyStopPin), emergencyStop, FALLING);
  pinMode(pushToTalkPin, INPUT_PULLUP); //define the other buttons as input w. pullup resistor
  pinMode(declinePin, INPUT_PULLUP);
  myVR.begin(9600); //beging voice recognition
  millisDelay(3000); //wait 3 seconds, non blocking delay function
  startup(); //startup function for voice recognition
  displayData(readBattVoltage()); //display battery reading
  lcd.setCursor(0,0); //print to lcd
  lcd.print("Ready");
  lcd.setCursor(0,1);
  lcd.print("At ");
  lcd.print(currentposition);
}

void loop(){
  lcd.setCursor(0,0); //print to lcd
  lcd.print("Ready");
  lcd.setCursor(0,1);
  lcd.print("At ");
  lcd.print(currentposition);
  
  newposition = String(printVR()); //read command from VR board
  if(digitalRead(pushToTalkPin) == LOW && currentposition != newposition && newposition != "!"){
    //if a new valid position is read, and the PTT button is pressed, ask for confirmation 
    lcd.clear();
    displayData(readBattVoltage());
    lcd.setCursor(0,0);
    lcd.print("Confirm?");
    lcd.setCursor(0,1);
    lcd.print(currentposition);
    lcd.print((char) 126);
    lcd.print(newposition);

    //wait for button release
    while(digitalRead(pushToTalkPin)==LOW){
      if(digitalRead(pushToTalkPin)){
        break;
        }
     }
     while(true){//wait for button press
        if(digitalRead(pushToTalkPin) == LOW){
          //if command is confirmed
          lcd.clear(); //print to lcd
          displayData(readBattVoltage());
          lcd.setCursor(0,0);
          lcd.print("Confirmed");
          millisDelay(2000);
          lcd.clear();
          displayData(readBattVoltage());          
          navigate(currentposition, newposition); //begin to navigate robot
          lcd.clear();
          displayData(readBattVoltage());    
          currentposition = newposition; //update position
          break;//exit loop
        }
        else if(digitalRead(declinePin) == LOW){
          //if command is declined
          lcd.clear();//print to lcd
          displayData(readBattVoltage());
          lcd.setCursor(0,0);
          lcd.print("Aborting");
          millisDelay(2000);
          lcd.clear();
          Serial.println("aborting");
          newposition = currentposition;//revert position
          break;//exit loop
        }
      }
  }
}

void turnLeft(){
  lcd.setCursor(0,1);//print to lcd
  lcd.print("Turn L     ");
  //set direction pins and move stepper
  digitalWrite(DIR2, HIGH);
  digitalWrite(DIR1, LOW);
  stepper.move(STEPLEFT);
}

void turnRight(){
  lcd.setCursor(0,1);//print to lcd
  lcd.print("Turn R     ");
  //set direction pins and move stepper
  digitalWrite(DIR1, HIGH);
  digitalWrite(DIR2, LOW);
  stepper.move(STEPRIGHT);
}

void goForward(int steps){
  //set direction pins
  digitalWrite(DIR1, HIGH);
  digitalWrite(DIR2, HIGH);
  stepper.move(steps); //move stepper 
  while(collision.distance() <= 8){
    //if object gets too close
    lcd.setCursor(0,1);
    lcd.print("Help me  ");
  }
}

int avoidObstacle(int obstacleDist){
  lcd.setCursor(0,1); //print to lcd
  lcd.print("Obstacle");
  //sweep corners
  int rightdist = front.cornerRight();
  int leftdist = front.cornerLeft();
  int width = ROBOTLENGTH;
  int obsLength = 0;
  bool dir; //0=right, 1=left //which direction it takes
  //turn left or right depending on which is shorter
  if(rightdist <= leftdist){
    turnRight();
    front.turnLeft();
    dir = 1;
  }
  else if(leftdist < rightdist){
    turnLeft();
    front.turnRight();
    dir = 0;
  }
  millisDelay(3000);
  
  //drive the width of the obstacle
  while(front.distance() <= obstacleDist+3){
    goForward(GRIDSTEP);
    width = width + GRIDSTEP; 
  }
  goForward(ROBOTLENGTH); // go forward
  millisDelay(2000);
  //turn depending on initial direction
  if(dir){
    turnLeft();
  }
  else{
    turnRight();
  }
  millisDelay(3000);
  goForward(FORWARDDIST); //drive forward
  millisDelay(2000);
  //read distance to object and drive forward
  int followdistance = front.distance();
  while (front.distance() <= followdistance+2){
    //drive as long as the obstacle is present
    goForward(GRIDSTEP);
    obsLength++;
    //log the length of the object
  }
  goForward(ROBOTLENGTH);
  millisDelay(2000);
  //turn robot and ir/servo
  if(dir){
    turnLeft();
    front.straight();
  }
  else{
    turnRight();
    front.straight();
  }
  millisDelay(3000);
  goForward(width); // move forward
  millisDelay(3000);
  //turn again
  if(!dir){
    turnLeft();
  }
  else{
    turnRight();
  }
  millisDelay(1000);
  //return traveled distance parallel to path
  return (FORWARDGRID + obsLength + ROBOTGRID);
}

int readBattVoltage(){
  // read analog value, convert to volt, convert to percent and return
  int voltReading = analogRead(battPin);
  int output = map(voltReading, 0, 1023, 0 ,500);
  int volts = map(output,335,470,0,100);
  return volts;
}

void displayData(int volts){
//display battery data on lcd
  lcd.clear();
  if ((volts >= 10) && (volts<100)){
    lcd.setCursor(12,0);
    lcd.write(byte(0));
    lcd.setCursor(13,0);
    lcd.print(volts);
    lcd.print("%");
  }else if(volts >= 100){
    lcd.setCursor(11,0);
    lcd.write(byte(0));
    lcd.setCursor(12,0);
    lcd.print(100);
    lcd.print("%");
  }else if (volts < 10){
    lcd.setCursor(13,0);
    lcd.write(byte(0));
    lcd.setCursor(14,0);
    lcd.print(volts);
    lcd.print("%");
   }
}

int nodeDistance(Coord current, Coord next){
  if(current.x == next.x){//if the y-value of current and next are the same, only the x value can be different
    return abs(current.y-next.y); // return the absolute value of the distance between current and next
  }
  else if(current.y == next.y){//if the y-value of current and next are the same, only the x value can be different
    return abs(current.x-next.x); // return the absolute value of the distance between current and next
  }
}

byte getHeading(Coord current, Coord next){
  //works under the assumption that all turns are 90 degrees and paths between helpnodes are perpendicular
  if(current.x == next.x){ //if the y-value of current and next are the same, only the x value can be different 
    if(next.y < current.y){ //if the next y is above the current y, the direction is north
      return 1; //north
    }
    else if(next.y > current.y){//if the next y is below the current y, the direction is south
      return 3; //south
    }
  }
  else if(current.y == next.y){//if the y-value of current and next are the same, only the x value can be different 
    if(next.x < current.x){//if the next x is left of the current x, the direction is west
      return 2; //west

    }
    else if(next.x > current.x){//if the next x is right of the current x, the direction is east
      return 4; //east
    }
  }
}

void drive(int griddistance){
//drive the number of gridcells in griddistance
  for(int i=0; i<griddistance; i++){
    lcd.setCursor(0,1);
    lcd.print("Drive ");
    lcd.print(griddistance);
    Serial.println(griddistance);
    goForward(GRIDSTEP);
    //for every cell, check for obstacle
    if(front.distance()<= MINDISTANCE){
      //if obstacle is present, skip distance returned by function
      i = i + avoidObstacle(front.distance());
    }
    millisDelay(500);
  }
}

void navigate(String startnode, String endnode){
  String printToDisplay = startnode + "→" + endnode;
  lcd.setCursor(0,0);
  lcd.print(startnode);
  lcd.print((char) 126);
  lcd.print(endnode);
  Path selectedpath; //the path between startnode and endnode
  Coord currentpos; //current position of robot
  Coord nextpos; //next position of robot/next position/node on path
  bool reverse = false; //whether to use the reversed or non-reversed path to navigate
  for(int i=0;i<pathsnumber;i++){ //determine which path in paths is the one that goes from startnode to endnode
    if(paths[i].wp1.wpName == startnode && paths[i].wp2.wpName == endnode){
      selectedpath = paths[i];
      reverse = false; //the non-reversed array of helpnodes must be used
      break;
    }
    else if(paths[i].wp2.wpName == startnode && paths[i].wp1.wpName == endnode){
      selectedpath = paths[i];
      reverse = true; //the reversed array must be used
      break;
    }
  }

  if(reverse == false){ //if the non-reversed array of help nodes must be used
    //startnode
        
    if(selectedpath.helpnodes[0].x != 0 && selectedpath.helpnodes[0].y != 0){
      currentpos = {selectedpath.wp1.x, selectedpath.wp1.y}; //set currentpos to startnode
      nextpos = {selectedpath.helpnodes[0].x, selectedpath.helpnodes[0].y}; //set nextpos to the first helpnode
    
      printroute(currentpos, nextpos, getHeading(currentpos, nextpos), nodeDistance(currentpos, nextpos)); //print the information needed to go from currentpos to nextpos - can be substituted with drive/turn functions on the robot
      //heading and distance comes from two function calls and passed directly to the printroute function
      turnDirection(getHeading(currentpos, nextpos));
      drive(nodeDistance(currentpos, nextpos));
    
      for(int i=0; i<MAX_HELPNODES-1; i++){
        if((selectedpath.helpnodes[i].x != 0 && selectedpath.helpnodes[i].y != 0) && (selectedpath.helpnodes[i+1].x != 0 && selectedpath.helpnodes[i+1].y != 0)){ //only use elements that are NOT {0,0}
          currentpos = selectedpath.helpnodes[i]; //set currentpos 
          nextpos = selectedpath.helpnodes[i+1];  //set nextpos
          printroute(currentpos, nextpos, getHeading(currentpos, nextpos), nodeDistance(currentpos, nextpos));//print the information
        
          turnDirection(getHeading(currentpos, nextpos));
          drive(nodeDistance(currentpos, nextpos));
      }
    }
  
    currentpos = nextpos; //the new currentpos is the same as the last nextpos in the above loop
    nextpos = {selectedpath.wp2.x, selectedpath.wp2.y}; //nextpos is the coordinates of the ending position
    printroute(currentpos, nextpos, getHeading(currentpos, nextpos), nodeDistance(currentpos, nextpos)); //print the final path from the final help node to the endnode
    
    turnDirection(getHeading(currentpos, nextpos));
    drive(nodeDistance(currentpos, nextpos));
  }
    else{
    currentpos = {selectedpath.wp1.x, selectedpath.wp1.y}; //set currentpos to startnode
    nextpos = {selectedpath.wp2.x, selectedpath.wp2.y}; //nextpos is the coordinates of the ending position      
    printroute(currentpos, nextpos, getHeading(currentpos, nextpos), nodeDistance(currentpos, nextpos)); //print the final path from the final help node to the endnode       
    
    turnDirection(getHeading(currentpos, nextpos));
    drive(nodeDistance(currentpos, nextpos));
    }
  }

  
  else{// same as above, just reversed
    //startnode
    if(selectedpath.helpnodes[0].x != 0 && selectedpath.helpnodes[0].y != 0){
    currentpos = {selectedpath.wp2.x, selectedpath.wp2.y};
    for(int i=0; i<MAX_HELPNODES; i++){
      if(selectedpath.helpnodesR[i].x != 0 && selectedpath.helpnodesR[i].y != 0){
        nextpos = {selectedpath.helpnodesR[i].x, selectedpath.helpnodesR[i].y};
        break;
      }
    }
    printroute(currentpos, nextpos, getHeading(currentpos, nextpos), nodeDistance(currentpos, nextpos));
    
    turnDirection(getHeading(currentpos, nextpos));
    drive(nodeDistance(currentpos, nextpos));
    
    for(int i=0; i<MAX_HELPNODES-1; i++){
      if((selectedpath.helpnodesR[i].x != 0 && selectedpath.helpnodesR[i].y != 0) && (selectedpath.helpnodesR[i+1].x != 0 && selectedpath.helpnodesR[i+1].y != 0)){
        currentpos = selectedpath.helpnodesR[i];
        nextpos = selectedpath.helpnodesR[i+1];
        printroute(currentpos, nextpos, getHeading(currentpos, nextpos), nodeDistance(currentpos, nextpos));
        turnDirection(getHeading(currentpos, nextpos));
        drive(nodeDistance(currentpos, nextpos));
      }
    }
      currentpos = nextpos;
      nextpos = {selectedpath.wp1.x, selectedpath.wp1.y};
      printroute(currentpos, nextpos, getHeading(currentpos, nextpos), nodeDistance(currentpos, nextpos));
      
      turnDirection(getHeading(currentpos, nextpos));
      drive(nodeDistance(currentpos, nextpos));
    }
    else{
      currentpos = {selectedpath.wp2.x, selectedpath.wp2.y};
      nextpos = {selectedpath.wp1.x, selectedpath.wp1.y};
      printroute(currentpos, nextpos, getHeading(currentpos, nextpos), nodeDistance(currentpos, nextpos));
      
      turnDirection(getHeading(currentpos, nextpos));
      drive(nodeDistance(currentpos, nextpos));
    }
  }
  Serial.println("done");
}

void printroute(Coord s, Coord e, char hdg, byte dist){
  //printing route info, for debugging
  Serial.print("From: ");
  Serial.print(s.x);
  Serial.print(",");
  Serial.print(s.y);
  Serial.print("\t");
  Serial.print("To: ");
  Serial.print(e.x);
  Serial.print(",");
  Serial.print(e.y);
  Serial.print("\t");
  Serial.print("Head:" );
  Serial.print(hdg, DEC);
  Serial.println("\t");
}

void turnDirection(byte newHdg){
  //make robot turn based on current and next heading
  //ghd = get heading
  //prevhd = previous heading
  //north=1, west=2, south=3, east=4
  int turn = hdg - newHdg;
  if(turn == -1 || turn == 3){
    turnLeft();
    millisDelay(500);
  }
  else if(turn == 1 || turn == -3){
    turnRight();
    millisDelay(500);
  }
  else if(abs(turn) == 2){
    turnRight();
    millisDelay(500);
    turnRight();
    millisDelay(500);
  }
  hdg = newHdg;   
}

void millisDelay(int period){
  //non blocking delay, just a waiting while loop
  unsigned long int prev = millis();
  while(millis()-prev <= period){
    
  }
}