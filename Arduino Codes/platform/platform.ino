#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <AccelStepper.h>

#define dirPin 2 //Digital output pin
#define stepPin 3 //Digital output pin
#define motorInterfaceType 1 //Easy driver interface
AccelStepper platform = AccelStepper(motorInterfaceType, stepPin, dirPin);
#define home_switch 9 //Micro-switch used for homing the platform
#define BAUD 57600
std_msgs::UInt16 led_msg;
ros::NodeHandle nh;

// Globals
int move_finished=1;  // Used to check if move is completed
long initial_homing=-1;  // Used to Home Stepper at startup
int steps = 200; // Steps per full revolution, step angle NEMA17 = 1.8 deg -> 200 steps per 360 deg

void subscriberCallback(const std_msgs::UInt16& led_msg) {
  if (led_msg.data  == 1) {
    StepperHoming();
    nh.loginfo("Werkt gewoon nie zeike");
   
  } else {
    nh.loginfo("Value other than 1");
  }
}

ros::Subscriber<std_msgs::UInt16> led_subscriber("toggle_led", &subscriberCallback);



void StepperHoming(){
  platform.setAcceleration(200.0);
  platform.setMaxSpeed(200.0);
  //Start Homing
  while (digitalRead(home_switch)) {  // Make the Stepper move CCW until the switch is activated   
    platform.moveTo(initial_homing);  // Set the position to move to
    initial_homing--;  // Decrease by 1 for next move if needed
    platform.run();  // Start moving the stepper
    delay(5);
  }
  //Switch is pressed,
  platform.setCurrentPosition(0);  // Set the current position as zero for now
  platform.setMaxSpeed(100.0);      // Set Max Speed of Stepper (Slower to get better accuracy)
  platform.setAcceleration(100.0);  // Set Acceleration of Stepper
  initial_homing=1;
  //Back off switch, untill it is not pressed
  while (!digitalRead(home_switch)) { // Make the Stepper move CW until the switch is deactivated
    platform.moveTo(initial_homing);  
    platform.run();
    initial_homing++;
    delay(5);
  }
  
  platform.setCurrentPosition(0); //Location where the switch is just deactivated

}

void setup()
{
  delay(1000);
  pinMode(home_switch, INPUT_PULLUP);
  //StepperHoming(); //Initial homing using the micro-switch
  platform.setMaxSpeed(1000.0);      // Max stepper speed
  platform.setAcceleration(1000.0);  // Max stepper acc
  nh.getHardware()->setBaud(BAUD);
  nh.initNode();
  nh.subscribe(led_subscriber);

}

void loop()
{
  nh.spinOnce();
  delay(1);
}
