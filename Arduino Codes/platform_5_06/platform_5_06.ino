#include <AccelStepper.h>
#include <math.h>
#define dirPin 2 //Digital output pin
#define stepPin 3 //Digital output pin
#define motorInterfaceType 1 //Easy driver interface
AccelStepper platform = AccelStepper(motorInterfaceType, stepPin, dirPin);
#define home_switch 9 //Micro-switch used for homing the platform
#define revolution 200 // Steps per full revolution, step angle NEMA17 = 1.8 deg -> 200 steps per 360 deg
#define radius 0.62 //Radius [cm] of the pulley mounted on the NEMA17, r = 2*d


// Globals
int move_finished = 1; // Used to check if move is completed
long initial_homing = -1; // Used to Home Stepper at startup
int input;
float cropX;
int platformRange = 6000; //Maximum steps travel range of the platfrom from homing position
float PixelX[] = {0.15,0.5,0.9}; //X-coordinate of pixels [cm] ([0] = Pixel 1 (most leftern pixel), [1] = Pixel 2 (middle pixel), [2] = Pixel 3 (closest to homing position))
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
    //MoveToCrop();
    break;
    case 4:
    MoveToCrop();
    
  default:
    break;
}
}

int Conversion(float X){
  // 100cm = 5030 steps // gemeten van 0 punt tot rechter punt platform.
  //Conversion of X coordinates [cm] to number of steps [-]
  int steps;
  int stepsperm = 5030; //lmao
  steps = X*stepsperm;
    Serial.print("Steps: ");
  Serial.println(steps);
  Serial.print("X: ");
  Serial.println(X);
  if(steps>6000){
    Serial.print("Out of bounds, limit to frame");
    return 5030;
    }
  else{
    return steps;
    }
  }

void MoveToPixel(){ //moves the platform to Pixel 1,2 or 3
  //temp:
  //platform.setSpeed(200);
  int pixel = input-1; //array index of pixel
  platform.moveTo(Conversion(PixelX[pixel]));
   platform.runToPosition();
  }

void MoveToCrop(){ //Moves the platform to the crop, based on the output of the mapping.py script
  
  Serial.println("Input the crop X-coordinate [m]: ");
  while (Serial.available() == 0) {
    // Wait for input
  }
  cropX = Serial.parseFloat(); //user inputs a value. Simulates subscribing to a topic with ROS
  int stepX = Conversion(cropX);
  Serial.print("Crop coordinate: "); //feedback of chosen value
  Serial.println(cropX);
  Serial.println(" ");
  if(stepX>platformRange||stepX<0){
    Serial.println("X-coordinate out of bounds");
    }
   else{
      platform.moveTo(stepX);
   platform.runToPosition();
      }
  
  }

void StepperHoming() { //Function that does the homing sequence
  platform.setAcceleration(300.0);
  platform.setMaxSpeed(300.0);
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
  platform.setMaxSpeed(2000.0);      // Max stepper speed
  platform.setAcceleration(1500.0);  // Max stepper acc
  Serial.println("Homing complete");
}

void setup()
{
  pinMode(home_switch, INPUT_PULLUP);
  //StepperHoming(); //Initial homing using the micro-switch
  
  Serial.begin(115200);
  delay(10);

}

void loop()
{
  Serial.println("Input any number: ");
  Serial.println("0 = Homing sequence");
  Serial.println("1 = Move to Pixel 1");
  Serial.println("2 = Move to Pixel 2");
  Serial.println("3 = Move to Pixel 3");
  Serial.println("4 = Move to coordinate");
  
  while (Serial.available() == 0) {
    // Wait for input
  }
  input = Serial.parseInt(); //user inputs a value. Simulates subscribing to a topic with ROS
  Serial.print("Chosen: "); //feedback of chosen value
  Serial.println(input);
  Serial.println(" ");
  
  if(input > 4){
    Serial.println("Invalid input");
    }
  else{
  subscriberCallback(); //If input is valid, run Callback function 
  }

}
