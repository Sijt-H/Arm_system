#include <AccelStepper.h>

//Pins X-direction
#define dirPinX 2 //Digital output pin
#define stepPinX 3 //Digital output pin
#define home_switchX 9 //Micro-switch used for homing the platform 

//Pins Y-direction
#define dirPinY 4 //Digital output pin
#define stepPinY 5 //Digital output pin
#define home_switchY 10 //Micro-switch used for homing the platform 


#define motorInterfaceType 1 //Easy driver interface
AccelStepper platform = AccelStepper(motorInterfaceType, stepPinX, dirPinX);
AccelStepper laser = AccelStepper(motorInterfaceType, stepPinY, dirPinY);
// Constants
#define revolution 200 // Steps per full revolution, step angle NEMA17 = 1.8 deg -> 200 steps per 360 deg
#define radius 0.62 //Radius [cm] of the pulley mounted on the NEMA17, r = 2*d
#define platformRange 6000 //Maximum steps travel range of the platfrom from homing position 
#define laserRange 503 //Maximum steps travel range of laser from homing position

// Globals
int move_finished = 1; // Used to check if move is completed
long initial_homing = -1; // Used to Home Stepper at startup
bool homing_completed = false;
int input;
float cropX;
float cropY;
float PixelX[] = {15,50,90}; //X-coordinate of pixels [cm] ([0] = Pixel 1 (most leftern pixel), [1] = Pixel 2 (middle pixel), [2] = Pixel 3 (closest to homing position))
//fix: convert to steps


//Function name is kept as it is in ROS
void subscriberCallback() {
 switch (input) {
  case 0:
    StepperHomingX(); //run homing sequence
    break;
    
  case 1:
    MoveToPixel();
    MoveToCropX();
    MoveToCropY();
    break;
    
     case 2:
    MoveToPixel();
    MoveToCropX();
    MoveToCropY();
    break;
    
     case 3:
    MoveToPixel();
    MoveToCropX();
    MoveToCropY();
    break;
    case 4:
    MoveToCropX(); //input custom amount to move in X direction
    case 5:
    MoveToCropY(); //input custom amount to move in Y direction
    case 9:
    StepperHomingY();
  default:
    break;
}
}
int ConversionY(float Y){
  // 100cm = 5030 steps // measured from homed 0 point to right side of the platform
  //ConversionX of X coordinates [cm] to number of steps [-]
  int steps;
  int stepsperm = 5030; //lmao
  steps = Y/100*stepsperm;
    Serial.print("Steps: ");
  Serial.println(steps);
  
  if(steps>platformRange){
    Serial.print("Out of bounds, limit to frame");
    return 0;
    }
  else{
    return steps;
    }
  }

int ConversionX(float X){
  // 100cm = 5030 steps // measured from homed 0 point to right side of the platform
  //ConversionX of X coordinates [cm] to number of steps [-]
  int steps;
  int stepsperm = 5030; //lmao
  steps = X/100*stepsperm;
    Serial.print("Steps: ");
  Serial.println(steps);
  
  if(steps>platformRange){
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
  platform.moveTo(ConversionX(PixelX[pixel]));
   platform.runToPosition();
  }

void MoveToCropY(){ //Moves the platform to the crop, based on the output of the mapping.py script
  
  Serial.println("Input the crop Y-coordinate [cm]: ");
  while (Serial.available() == 0) {
    // Wait for input
  }
  cropY = Serial.parseFloat(); //user inputs a value. Simulates subscribing to a topic with ROS
  int stepY = ConversionY(cropY);
  Serial.print("Crop coordinate: "); //feedback of chosen value
  Serial.println(cropY);
  Serial.println(" ");
  if(stepY>laserRange||stepY<0){
    Serial.println("X-coordinate out of bounds");
    }
   else{
      laser.moveTo(stepY);
   laser.runToPosition();
      }
  
  }

void MoveToCropX(){ //Moves the platform to the crop, based on the output of the mapping.py script
  
  Serial.println("Input the crop X-coordinate [cm]: ");
  while (Serial.available() == 0) {
    // Wait for input
  }
  cropX = Serial.parseFloat(); //user inputs a value. Simulates subscribing to a topic with ROS
  int stepX = ConversionX(cropX);
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

void StepperHomingY() { //Function that does the homing sequence
  laser.setAcceleration(100.0);
  laser.setMaxSpeed(100.0);
  //Start Homing
  while (digitalRead(home_switchY)) {  // Make the Stepper move CW until the switch is activated
    laser.moveTo(initial_homing);  // Set the position to move to
    initial_homing--;  // Decrease by 1 for next move if needed
    laser.run();  // Start moving the stepper
    delay(5);
  }
  //Switch is pressed,
  laser.setCurrentPosition(0);  // Set the current position as zero for now
  laser.setMaxSpeed(100.0);      // Set Max Speed of Stepper (Slower to get better accuracy)
  laser.setAcceleration(100.0);  // Set Acceleration of Stepper
  initial_homing = 1;
  //Back off switch, untill it is not pressed
  while (!digitalRead(home_switchY)) { // Make the Stepper move CCW until the switch is deactivated
    laser.moveTo(initial_homing);
    laser.run();
    initial_homing++;
    delay(5);
  }

  laser.setCurrentPosition(0); //Location where the switch is just deactivated: laser.X=0
  laser.setMaxSpeed(500.0);      // Max stepper speed
  laser.setAcceleration(500.0);  // Max stepper acc
  Serial.println("Homing complete");
}


void StepperHomingX() { //Function that does the homing sequence
  platform.setAcceleration(300.0);
  platform.setMaxSpeed(300.0);
  //Start Homing
  while (digitalRead(home_switchX)) {  // Make the Stepper move CW until the switch is activated
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
  while (!digitalRead(home_switchX)) { // Make the Stepper move CCW until the switch is deactivated
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
  pinMode(home_switchX, INPUT_PULLUP);
  Serial.begin(115200);
  delay(10);

  //after every restart of the Mega, a homing sequence needs to be done
  Serial.println("Type 0 to start X-axis homing sequence: ");
  int homing_confirm = 2;
  while (homing_confirm != 0){
    while (Serial.available() == 0) {
    // Wait for input
  }
  homing_confirm = Serial.parseInt(); //user inputs a value to confirm homing sequence initiation
    
    }
    StepperHomingX(); 
    homing_completed = true;

  
    Serial.println("Type 9 to start Y-axis homing sequence: ");
  homing_confirm = 2;
  while (homing_confirm != 0){
    while (Serial.available() == 0) {
    // Wait for input
  }
  homing_confirm = Serial.parseInt(); //user inputs a value to confirm homing sequence initiation
    
    }
    StepperHomingY(); 
    homing_completed = true;
}

void loop()
{
  Serial.println("-----------------------------");
  Serial.println("Menu: ");
  Serial.println("0 = Homing sequence X-axis");
  Serial.println("1 = Move to Pixel 1");
  Serial.println("2 = Move to Pixel 2");
  Serial.println("3 = Move to Pixel 3");
  Serial.println("4 = Move to X-coordinate");
  Serial.println("5 = Move to Y-coordinate");
  Serial.println("9 = Homing sequence Y-axis");
  
  while (Serial.available() == 0) {
    // Wait for input
  }
  input = Serial.parseInt(); //user inputs a value. Simulates subscribing to a topic with ROS
  Serial.print("Chosen: "); //feedback of chosen value
  Serial.println(input);
  Serial.println(" ");
  
  if(input > 5 || input != 9){ //if not any of the given options
    Serial.println("Invalid input");
    }
  else{
  subscriberCallback(); //If input is valid, run Callback function 
  }

}
