#include "roombaDefines.h"
#include <SoftwareSerial.h>

//RXTX Connections
int rxPin = 10;
int txPin = 11;
SoftwareSerial Roomba(rxPin, txPin);
 
//#include "WProgram.h" //include the Arduino library
#include <stdlib.h>
 
 
#include <ros.h>
#include "std_msgs/Int32MultiArray.h"
 

//include Wire/ twi for the BlinkM
#include <Wire.h>
 extern "C" { 
#include "utility/twi.h" 
 }

/*
 * Send data to the move subscrber. should be a string with 4 characthers
 * first charachter is speed in mm/s
 * second """"" is direction (zero for forward, one for reverse)
 * third """""" is radius in mm. set to zero for straight and 1 for turn on spot 
 * fourth """" is turn direction (zero for something one for the otherone) 
 */
void move_cb( const std_msgs::Int32MultiArray& move_cmd ){
  int vRight = move_cmd->data[0];
  int vLeft = move_cmd->data[1];
  driveWheels(vRight, vLeft);     
  //setLED(solid, color);
}


ros::NodeHandle nh;
ros::Subscriber<std_msgs::String> sub("move" , move_cb);


void setup() {
  Roomba.begin(19200);
  Serial.begin(19200);

  pinMode(ddPin, OUTPUT);
  pinMode(ledPin, OUTPUT);

  delay(2000);
  Serial.print("Roomba Remote Control\n");

  wakeUp();
  startSafe();

  // turn-off all LEDs
  setPowerLED(128, 0);
  setDebrisLED(OFF);
  setDockLED(OFF);
  setSpotLED(OFF);
  setWarningLED(OFF);
  cleanDigitLED();

  playSound(1); // start sound
  while (digitalRead(buttonPin))
  {
    setDebrisLED(ON);
    writeLEDs('-','-','-','-');
  }
  setDebrisLED(OFF);
  writeLEDs('r','e','d','y');

  //playSound(2);

  //motorSquareTest();

  nh.initNode();
  nh.subscribe(sub);
}

void loop() {

  /*if(Serial.available()){
    command = Serial.read();
    Serial.print("Command: " );
    Serial.println(command);
  }
  manualCmd();
  */
  nh.spinOnce();
  delay(1);
}
