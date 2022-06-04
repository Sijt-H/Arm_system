#include <AccelStepper.h>

#define dirPin 2 //Digital output pin
#define stepPin 3 //Digital output pin
#define motorInterfaceType 1 //Easy driver interface
AccelStepper platform = AccelStepper(motorInterfaceType, stepPin, dirPin);
#define home_switch 9 //Micro-switch used for homing the platform

// Globals
int move_finished = 1; // Used to check if move is completed
long initial_homing = -1; // Used to Home Stepper at startup
int steps = 200; // Steps per full revolution, step angle NEMA17 = 1.8 deg -> 200 steps per 360 deg
int input;
int cropX;
int platformRange = 130; //Maximum travel range of the platfrom from homing position
int PixelX[] = {30,80,120}; //X-coordinate of pixels ([0] = Pixel 1 (most leftern pixel), [1] = Pixel 2 (middle pixel), [2] = Pixel 3 (closest to homing position))
//fix: convert to steps


//Function name is kept as it is in ROS
void subscriberCallback() {
 switch (input) {
  case 0:
    StepperHoming(); //run homing sequence
    break;
    
  case 1:
    MoveToPixel();
    MoveToCrop();
    break;
    
     case 2:
    MoveToPixel();
    MoveToCrop();
    break;
    
     case 3:
    MoveToPixel();
    MoveToCrop();
    break;
    
  default:
    break;
}
}

void MoveToPixel(){ //moves the platform to Pixel 1,2 or 3
  //temp:
  int pixel = input-1; //array index of pixel
  platform.moveTo(PixelX[pixel]); //move to pixelX location
  platform.run();
  }

void MoveToCrop(){ //Moves the platform to the crop, based on the output of the mapping.py script
  
  Serial.println("Input the crop X-coordinate: ");
  while (Serial.available() == 0) {
    // Wait for input
  }
  cropX = Serial.parseInt(); //user inputs a value. Simulates subscribing to a topic with ROS
  
  Serial.print("Chosen: "); //feedback of chosen value
  Serial.println(cropX);
  Serial.println(" ");
  if(cropX>platformRange){
    Serial.println("X-coordinate out of bounds");
    }
  
  }

void StepperHoming() { //Function that does the homing sequence
  platform.setAcceleration(200.0);
  platform.setMaxSpeed(200.0);
  //Start Homing
  while (digitalRead(home_switch)) {  // Make the Stepper move CW until the switch is activated
    platform.moveTo(initial_homing);  // Set the position to move to
    initial_homing--;  // Decrease by 1 for next move if needed
    platform.run();  // Start moving the stepper
    delay(5);
  }
  //Switch is pressed,
  platform.setCurrentPosition(0);  // Set the current position as zero for now
  platform.setMaxSpeed(100.0);      // Set Max Speed of Stepper (Slower to get better accuracy)
  platform.setAcceleration(100.0);  // Set Acceleration of Stepper
  initial_homing = 1;
  //Back off switch, untill it is not pressed
  while (!digitalRead(home_switch)) { // Make the Stepper move CCW until the switch is deactivated
    platform.moveTo(initial_homing);
    platform.run();
    initial_homing++;
    delay(5);
  }

  platform.setCurrentPosition(0); //Location where the switch is just deactivated: Platform.X=0
  Serial.println("Homing complete");
}

void setup()
{
  pinMode(home_switch, INPUT_PULLUP);
  //StepperHoming(); //Initial homing using the micro-switch
  platform.setMaxSpeed(1000.0);      // Max stepper speed
  platform.setAcceleration(1000.0);  // Max stepper acc
  Serial.begin(57600);
  delay(10);

}

void loop()
{
  Serial.println("Input any number: ");
  Serial.println("0 = Homing sequence");
  Serial.println("1 = Move to Pixel 1");
  Serial.println("2 = Move to Pixel 2");
  Serial.println("3 = Move to Pixel 3");
  while (Serial.available() == 0) {
    // Wait for input
  }
  input = Serial.parseInt(); //user inputs a value. Simulates subscribing to a topic with ROS
  
  Serial.print("Chosen: "); //feedback of chosen value
  Serial.println(input);
  Serial.println(" ");
  
  if(input > 3){
    Serial.println("Invalid input");
    }
  else{
  subscriberCallback(); //If input is valid, run Callback function 
  }

}
