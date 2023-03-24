/*  
 * Arduino code to send and receive I2C data 
 * 
 * This code contains the following features:
 * 
 * Constant Speed Commands for all motors and bins
 * 
 * Basic Homing Commands to End Stops (Trolley)
 * 
 * Accel Motor Commands for each motor
 * Accel Motor Commands and constant speed until limit switch for trolley motor
 * Sleep settings for motors (Troley Accel and Endstops)
 * 
 * Load Cell Commands (unsucessfully tested in the cabinet)
 *  
 * SDA <--> SDA 
 * SCL <--> SCL 
 * GND <--> GND 
 *  
 * Sets built-in LED (1 = on, 0 = off) on Feather when requested  
 * and responds with data received 
 */  
 

#include <AccelStepper.h>
 
#include <Wire.h>  
#define SLAVE_ADDRESS 0x54       // I2C address for Arduino  
#define LED 13                   // Built-in LED  
int i2cData = 0;                 // the I2C data received  

#include "HX711.h" 

// Define pin connections & motor's steps per revolution
const int dirPinA = 34;
const int stepPinA = 33;
const int sleepPinA = 32;
const int stepsPerRevolutionA = 650;
const int dirPinB = 37;
const int stepPinB = 36;
const int sleepPinB = 35;
const int stepsPerRevolutionB = 1000;
const int stepsPerRevolution2B = 1990;
const int dirPinC = 40;
const int stepPinC = 39;
const int sleepPinC = 38;
const int stepsPerRevolutionC = 1700;
const int dirPinD = 43;
const int stepPinD = 42;
const int sleepPinD = 41;
const int stepsPerRevolutionD = 1000;
int dirPin;
int stepPin;
int total_steps;
int steps_delay;
int step_direction;

// Declaring Limit Switches
int LS_left_trolley = 29;
int LS_right_trolley = 23;
int LS_open_trap = 26;
int LS_close_trap = 27;
int LS_right_rfid = 30;
int LS_left_rfid = 22;
int Time_out_pin = 28;

int LS_right_rfid_last_touched = 1;
int LS_left_rfid_last_touched = 0;



const int calibration_weight = 123; // Set mass of calibration weight which will automatically reset scale when called.
float last_weight = 0; // last weight used to check floating load cell and for taring.  
float calibrated_scale = 196618/123; // Calibration scale to be used with HX711 and load cell

int SensorValue = 0;
int LaseredSensorValue = 0;
int laser_trigger_pin = 52;
int sensorPin = A0; // select input pin for LDR

// Define motor interface type
#define motorInterfaceType 1

// Creates an instance
AccelStepper myStepperA(motorInterfaceType, stepPinA, dirPinA);
// Creates an instance
AccelStepper myStepperB(motorInterfaceType, stepPinB, dirPinB);
// Creates an instance
AccelStepper myStepperC(motorInterfaceType, stepPinC, dirPinC);
// Creates an instance
AccelStepper myStepperD(motorInterfaceType, stepPinD, dirPinD);


// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 24;
const int LOADCELL_SCK_PIN = 25;

int weightmeasurement = 0;

char command = 0;
char response = 0;


HX711 scale;

void setup(){  
  Wire.begin(SLAVE_ADDRESS);  
  Wire.onReceive(receiveData);  
  Wire.onRequest(sendData); 
    
// Declare pins as Outputs
  // set the maximum speed, acceleration factor,
  // initial speed and the target position
//  myStepper.setMaxSpeed(1000);
//  myStepper.setAcceleration(3000);
//  myStepper.setSpeed(600);
//
//  myStepper.move(-200);  
//  // Move the motor one step
//  myStepper.run();
//
//  myStepper.move(200);  
//  // Move the motor one step
//  myStepper.run();

  pinMode(stepPinA, OUTPUT);
  pinMode(dirPinA, OUTPUT);
  pinMode(sleepPinA, OUTPUT);
  pinMode(stepPinB, OUTPUT);
  pinMode(dirPinB, OUTPUT);
  pinMode(sleepPinB, OUTPUT);
  pinMode(stepPinC, OUTPUT);
  pinMode(dirPinC, OUTPUT);
  pinMode(sleepPinC, OUTPUT);
  pinMode(stepPinD, OUTPUT);
  pinMode(dirPinD, OUTPUT);
  pinMode(sleepPinD, OUTPUT);

  //declaring limit switches
  pinMode(LS_left_trolley, INPUT);
  pinMode(LS_right_trolley, INPUT);
  pinMode(LS_right_rfid, INPUT);
  pinMode(LS_left_rfid, INPUT);
  pinMode(LS_open_trap, INPUT);
  pinMode(LS_close_trap, INPUT);

  pinMode(Time_out_pin, INPUT);

  // Spin motor slowly
  digitalWrite(dirPinA, HIGH);
  for(int x = 0; x < stepsPerRevolutionA; x++)
  {
    digitalWrite(stepPinA, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPinA, LOW);
    delayMicroseconds(1000);
  }

  delay(1000);

    // Set motor direction clockwise
  digitalWrite(dirPinA, LOW);

  // Spin motor slowly
  for(int x = 0; x < stepsPerRevolutionA; x++)
  {
    digitalWrite(stepPinA, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPinA, LOW);
    delayMicroseconds(1000);
  }
      
      Serial.begin(38400);
  Serial.println("HX711 Demo");

  Serial.println("Initializing the scale");

  // Initialize library with data output pin, clock input pin and gain factor.
  // Channel selection is made by passing the appropriate gain:
  // - With a gain factor of 64 or 128, channel A is selected
  // - With a gain factor of 32, channel B is selected
  // By omitting the gain factor parameter, the library
  // default "128" (Channel A) is used here.
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

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

  scale.set_scale(65766/123);                      // this value is obtained by calibrating the scale with known weights; see the README for details
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
}


void loop() { 
  run_command();
  delay(100);
  // Everything happens in the interrupts
}

// Handle reception of incoming I2C data  
void receiveData(int byteCount) {  
  while (Wire.available()) {  
    command = Wire.read(); // remember the command for when we get request
  } // End of receive event
}  
// Handle request to send I2C data  
void sendData() {   
  Wire.write(response);
  if (response != 0){
    response = 0;
  } 
  }
  


// command list;
void run_command()
  {
   if (command == 0) {
    }

   else if (command == 1){
    dirPin = dirPinA;
    stepPin = stepPinA;
    step_direction = HIGH;
    total_steps = stepsPerRevolutionA;
    steps_delay = 1000;
    run_motor();
    command = 0;
    response = 1;}
    
   else if (command == 2){
    dirPin = dirPinA;
    stepPin = stepPinA;
    step_direction = LOW;
    total_steps = stepsPerRevolutionA;
    steps_delay = 1000;
    run_motor();
    command = 0;
    response = 1;}

   else if (command == 3){
    dirPin = dirPinB;
    stepPin = stepPinB;
    step_direction = HIGH;
    total_steps = stepsPerRevolutionB;
    steps_delay = 1000;
    run_motor();
    command = 0;
    response = 1;}

   else if (command == 4){
    dirPin = dirPinB;
    stepPin = stepPinB;
    step_direction = LOW;
    total_steps = stepsPerRevolutionB;
    steps_delay = 1000;
    run_motor();
    command = 0;
    response = 1;}

   else if (command == 5){
    dirPin = dirPinC;
    stepPin = stepPinC;
    step_direction = HIGH;
    total_steps = stepsPerRevolutionC;
    steps_delay = 600;
    run_motor();
    command = 0;
    response = 1;}

   else if (command == 6){
    dirPin = dirPinC;
    stepPin = stepPinC;
    step_direction = LOW;
    total_steps = stepsPerRevolutionC;
    steps_delay = 600;
    run_motor();
    command = 0;
    response = 1;}

   // New Accell Library commands
   // RFID Motor opening hole
   else if (command == 7){
    myStepperA.setCurrentPosition(0); //Reseting Current Position back to zero
    myStepperA.setMaxSpeed(800);
    myStepperA.setAcceleration(1000);
    myStepperA.runToNewPosition(stepsPerRevolutionA);
    command = 0; //Resetting Command to Blank command after completion
    response = 1;}

   // RFID Motor Closing hole and returning RFID reader to porch position
   else if (command == 8){
    myStepperA.setCurrentPosition(0); //Reseting Current Position back to zero
    myStepperA.setMaxSpeed(800);
    myStepperA.setAcceleration(1000);
    myStepperA.runToNewPosition(-stepsPerRevolutionA);
    command = 0; //Resetting Command to Blank command after completion
    response = 1;}

   // Trolley Motor to Quarantine Bin from Home Position
   else if (command == 9){
    myStepperB.setCurrentPosition(0); //Reseting Current Position back to zero
    myStepperB.setMaxSpeed(1000);
    myStepperB.setAcceleration(3000);
    myStepperB.runToNewPosition(stepsPerRevolutionB);
    command = 0; //Resetting Command to Blank command after completion
    response = 1;}

   // Trolley Motor from quarantine bin back to Home Position
   else if (command == 10){
    myStepperB.setCurrentPosition(0); //Reseting Current Position back to zero
    myStepperB.setMaxSpeed(1000);
    myStepperB.setAcceleration(3000);
    myStepperB.runToNewPosition(-stepsPerRevolutionB);
    command = 0; //Resetting Command to Blank command after completion
    response = 1;}

   // Trolley Motor to Bin B from home position
   else if (command == 11){
    myStepperB.setCurrentPosition(0); //Reseting Current Position back to zero
    myStepperB.setMaxSpeed(1000);
    myStepperB.setAcceleration(3000);
    myStepperB.runToNewPosition(stepsPerRevolution2B);
    command = 0; //Resetting Command to Blank command after completion
    response = 1;}

   // Trolley Motor return from Bin B; back to Home position
   else if (command == 12){
    myStepperB.setCurrentPosition(0); //Reseting Current Position back to zero
    myStepperB.setMaxSpeed(800);
    myStepperB.setAcceleration(3000);
    myStepperB.runToNewPosition(-stepsPerRevolution2B);
    command = 0; //Resetting Command to Blank command after completion
    response = 1;}

   // Trap Door Motor Open
   else if (command == 13){
    myStepperC.setCurrentPosition(0); //Reseting Current Position back to zero
    myStepperC.setMaxSpeed(800);
    myStepperC.setAcceleration(2000);
    myStepperC.runToNewPosition(stepsPerRevolutionC);
    command = 0; //Resetting Command to Blank command after completion
    response = 1;}

   // Trap Door Motor Close
   else if (command == 14){
    myStepperC.setCurrentPosition(0); //Reseting Current Position back to zero
    myStepperC.setMaxSpeed(800);
    myStepperC.setAcceleration(2000);
    myStepperC.runToNewPosition(-stepsPerRevolutionC);
    command = 0; //Resetting Command to Blank command after completion
    response = 1;}

   // RFID Motor - Accel Stepper With Constant Speed to Limit Switch
   else if (command == 15){
    digitalWrite(sleepPinA, HIGH);
    myStepperA.setCurrentPosition(0); //Reseting Current Position back to zero
    myStepperA.setMaxSpeed(1000);
    myStepperA.setAcceleration(3000);
    myStepperA.runToNewPosition(stepsPerRevolutionA);
    stepperA_home_forward_command(LS_right_rfid);
    digitalWrite(sleepPinA, LOW);
    command = 0; //Resetting Command to Blank command after completion
    if (digitalRead(Time_out_pin) != HIGH){
      response = 2;
      }
    else {
      response = 1;
      }
    }

   // RFID Motor Reversed - Accel Stepper
   else if (command == 16){
    digitalWrite(sleepPinA, HIGH);
    myStepperA.setCurrentPosition(0); //Reseting Current Position back to zero
    myStepperA.setMaxSpeed(1000);
    myStepperA.setAcceleration(3000);
    myStepperA.runToNewPosition(-stepsPerRevolutionA);
    stepperA_home_reverse_command(LS_left_rfid);
    digitalWrite(sleepPinA, LOW);
    command = 0; //Resetting Command to Blank command after completion
    if (digitalRead(Time_out_pin) != HIGH){
      response = 2;
      }
    else {
      response = 1;
      }
     }

  // RFID Motor - Accel Stepper With Constant Speed to Limit Switch
   else if (command == 115){
    digitalWrite(sleepPinA, HIGH);
    myStepperA.setCurrentPosition(0); //Reseting Current Position back to zero
    myStepperA.setMaxSpeed(1000);
    myStepperA.setAcceleration(3000);
    myStepperA.runToNewPosition(stepsPerRevolutionA);
    while (digitalRead(LS_right_rfid) == HIGH){
      stepperA_home_forward_command_no_timeout(LS_right_rfid);
      }
    digitalWrite(sleepPinA, LOW);
    command = 0; //Resetting Command to Blank command after completion
    response = 2;
    }

   // RFID Motor Reversed - Accel Stepper
   else if (command == 116){
    digitalWrite(sleepPinA, HIGH);
    myStepperA.setCurrentPosition(0); //Reseting Current Position back to zero
    myStepperA.setMaxSpeed(2000);
    myStepperA.setAcceleration(3000);
    myStepperA.runToNewPosition(-stepsPerRevolutionA);
    while (digitalRead(LS_left_rfid) == HIGH){
      stepperA_home_reverse_command_no_timeout(LS_left_rfid);
      }
    digitalWrite(sleepPinA, LOW);
    command = 0; //Resetting Command to Blank command after completion
    response = 2;
     }

   // Trolley Motor Bin C - Accel Stepper With Constant Speed to Limit Switch
   else if (command == 17){
    digitalWrite(sleepPinB, HIGH);
    myStepperB.setCurrentPosition(0); //Reseting Current Position back to zero
    myStepperB.setMaxSpeed(1000);
    myStepperB.setAcceleration(3000);
    myStepperB.runToNewPosition(stepsPerRevolution2B);
    stepperB_home_forward_command(LS_right_trolley);
    digitalWrite(sleepPinB, LOW);
    command = 0; //Resetting Command to Blank command after completion
    if (digitalRead(Time_out_pin) != HIGH){
      response = 2;
      }
    else {
      response = 1;
      }
    }

   // Trolley Motor Bin C Reversed - Accel Stepper With Constant Speed to Limit Switch
   else if (command == 18){
    digitalWrite(sleepPinB, HIGH);
    myStepperB.setCurrentPosition(0); //Reseting Current Position back to zero
    myStepperB.setMaxSpeed(1000);
    myStepperB.setAcceleration(3000);
    myStepperB.runToNewPosition(-stepsPerRevolution2B);
    stepperB_home_reverse_command(LS_left_trolley);
    digitalWrite(sleepPinB, LOW);
    command = 0; //Resetting Command to Blank command after completion
    if (digitalRead(Time_out_pin) != HIGH){
      response = 2;
      }
    else {
      response = 1;
      }
    }

   // Trolley Motor Bin Q Reversed - Accel Stepper With Constant Speed to Limit Switch
   else if (command == 19){
    digitalWrite(sleepPinB, HIGH);
    myStepperB.setCurrentPosition(0); //Reseting Current Position back to zero
    myStepperB.setMaxSpeed(1000);
    myStepperB.setAcceleration(3000);
    myStepperB.runToNewPosition(-stepsPerRevolutionB);
    stepperB_home_reverse_command(LS_left_trolley);
    digitalWrite(sleepPinB, LOW);
    command = 0; //Resetting Command to Blank command after completion
    if (digitalRead(Time_out_pin) != HIGH){
      response = 2;
      }
    else {
      response = 1;
      }
    }

   // Trap Door Motor Open - Accel Stepper With Constant Speed to Limit Switch
   else if (command == 20){
    digitalWrite(sleepPinC, HIGH);
    myStepperC.setCurrentPosition(0); //Reseting Current Position back to zero
    myStepperC.setMaxSpeed(800);
    myStepperC.setAcceleration(2000);
    myStepperC.runToNewPosition(stepsPerRevolutionC);
    stepperC_home_forward_command(LS_open_trap);
    digitalWrite(sleepPinC, LOW);
    command = 0; //Resetting Command to Blank command after completion
    if (digitalRead(Time_out_pin) != HIGH){
      response = 2;
      }
    else {
      response = 1;
      }
    }

   // Trap Door Motor Close- Accel Stepper With Constant Speed to Limit Switch
   else if (command == 21){
    digitalWrite(sleepPinC, HIGH);
    myStepperC.setCurrentPosition(0); //Reseting Current Position back to zero
    myStepperC.setMaxSpeed(800);
    myStepperC.setAcceleration(2000);
    myStepperC.runToNewPosition(-stepsPerRevolutionC);
    stepperC_home_reverse_command(LS_close_trap);
    digitalWrite(sleepPinC, LOW);
    command = 0; //Resetting Command to Blank command after completion
    if (digitalRead(Time_out_pin) != HIGH){
      response = 2;
      }
    else {
      response = 1;
      }
    }

   // Load Cell Calibration Command 1
   else if (command == 22){
    //scale.power_up();
    scale.set_scale(); //Calling to start calibration
    scale.tare();
    Serial.println("Scare Tared");
    scale.set_scale(calibrated_scale);
    Serial.println("New scale calibrated");
    response = 1;
    //scale.power_down();
    command = 0; // resetting command
    }

   // Load Cell Calibration Command 2
   else if (command == 23){
    //scale.power_up();
    Serial.println("Scale calibrating units with a known weight");
    float avg_units = scale.get_value(10); // intialising new value 'avg_units' and Taking 10 measurements of units
    calibrated_scale = avg_units/calibration_weight;
    scale.set_scale(calibrated_scale);
    response = 1;
    //scale.power_down();
    command = 0; // resetting command
    }

   // Load Cell Calibration Complete tare and scale
   else if (command == 33){
    //scale.power_up();
    delay(100);
    scale.set_scale(); //Calling to start calibration
    scale.tare(10);
    Serial.println("put cup on scale");
    delay(3000);
    float avg_units = scale.get_value(20); // intialising new value 'avg_units' and Taking 10 measurements of units
    calibrated_scale = avg_units/calibration_weight;
    scale.set_scale(calibrated_scale);
    response = 1;
    command = 0; // resetting command
    //scale.power_down();
    }

   // Take weight measurement
   else if (command == 24){
    //scale.power_up();
    delay(100);
    scale.set_scale(calibrated_scale);
    Serial.print("get value command 24: \t\t");
    Serial.println(scale.get_value(5));
    Serial.println(scale.get_units(5));
    float weight = scale.get_units(5);
    if (weight < calibration_weight+10 && weight > calibration_weight-10){
      Serial.println("Correct cup weight identified");
      response = 2;
      //scale.power_down();
      }
    else if (weight > calibration_weight +10){
      Serial.println("Incorrect cup weight identified: Overweight");
      response = 3;
      //scale.power_down();
      }
    else if (weight < calibration_weight -10){
      Serial.println("Incorrect cup weight identified: Underweight");
      response = 4;
      //scale.power_down();
      }
    command = 0; // resetting command
    }

   // Take weight measurement
   else if (command == 34){
    scale.power_up();
    Serial.print("get value: \t\t");
    Serial.println(scale.get_value(5));
    Serial.println(scale.get_units(5));
    scale.set_scale(196618/123);
    float weight = scale.get_units(5);
    if (weight < last_weight+10 && weight > last_weight-10){
      response = 2;
      scale.power_down();
      }
    else {
      response = 1;
      scale.power_down();
      }
    command = 0; // resetting command
    }

//   // RFID Motor - Constant Speed to Limit Switch
//   else if (command == 25){
//    digitalWrite(sleepPinA, HIGH);
//    stepperA_home_forward_command(LS_right_rfid);
//    digitalWrite(sleepPinA, LOW);
//    command = 0; //Resetting Command to Blank command after completion
//    if (digitalRead(Time_out_pin) != HIGH){
//      response = 2;
//      }
//    else {
//      response = 1;
//      }
//    }

   // Trolley Motor Bin A - Constant Speed to Limit Switch
   else if (command == 25){
    digitalWrite(sleepPinA, HIGH);
    stepperA_home_forward_command(LS_right_rfid);
    digitalWrite(sleepPinA, LOW);
    command = 0; //Resetting Command to Blank command after completion
    if (digitalRead(Time_out_pin) != HIGH){
      response = 2;
      }
    else {
      response = 1;
      }
    }

   // RFID Motor - Reverse Constant Speed to Limit Switch
   else if (command == 26){
    digitalWrite(sleepPinA, HIGH);
    stepperA_home_reverse_command(LS_left_rfid);
    digitalWrite(sleepPinA, LOW);
    command = 0; //Resetting Command to Blank command after completion
    if (digitalRead(Time_out_pin) != HIGH){
      response = 2;
      }
    else {
      response = 1;
      }
    }

   // Trolley Motor Bin C - Constant Speed to Limit Switch
   else if (command == 27){
    digitalWrite(sleepPinB, HIGH);
    stepperB_home_forward_command(LS_right_trolley);
    digitalWrite(sleepPinB, LOW);
    command = 0; //Resetting Command to Blank command after completion
    if (digitalRead(Time_out_pin) != HIGH){
      response = 2;
      }
    else {
      response = 1;
      }
    }

   // Trolley Motor Bin C - Constant Speed to Limit Switch
   else if (command == 28){
    digitalWrite(sleepPinB, HIGH);
    stepperB_home_reverse_command(LS_left_trolley);
    digitalWrite(sleepPinB, LOW);
    command = 0; //Resetting Command to Blank command after completion
    if (digitalRead(Time_out_pin) != HIGH){
      response = 2;
      }
    else {
      response = 1;
      }
    }

   // Trap Door Motor Open - Constant Speed to Limit Switch
   else if (command == 29){
    digitalWrite(sleepPinC, HIGH);
    stepperC_home_forward_command(LS_open_trap);
    digitalWrite(sleepPinC, LOW);
    command = 0; //Resetting Command to Blank command after completion
    if (digitalRead(Time_out_pin) != HIGH){
      response = 2;
      }
    else {
      response = 1;
      }
    }

   // Trap Door Motor Open - Constant Speed to Limit Switch
   else if (command == 30){
    digitalWrite(sleepPinC, HIGH);
    stepperC_home_reverse_command(LS_close_trap);
    digitalWrite(sleepPinC, LOW);
    command = 0; //Resetting Command to Blank command after completion
    if (digitalRead(Time_out_pin) != HIGH){
      response = 2;
      }
    else {
      response = 1;
      }
    }

    // Sliding Door Open Motor Command
   else if (command == 41){
    digitalWrite(sleepPinD, HIGH);
    myStepperD.setCurrentPosition(0); //Reseting Current Position back to zero
    myStepperD.setMaxSpeed(1400);
    myStepperD.setAcceleration(1000);
    myStepperD.runToNewPosition(4*stepsPerRevolutionD);
    digitalWrite(sleepPinD, LOW);
    command = 0; //Resetting Command to Blank command after completion
    response = 1;}

   else if (command == 51){
    SensorValue = analogRead(sensorPin);
    Serial.print("Value before laser: ");
    Serial.println(SensorValue);// read the value from the sensor
    digitalWrite(laser_trigger_pin, HIGH);
    delay(200);
    LaseredSensorValue = analogRead(sensorPin);
    Serial.print("Value after laser: ");
    Serial.println(LaseredSensorValue);// read the value from the sensor
    Serial.println("-----------");
    digitalWrite(laser_trigger_pin, LOW);
    delay(100);
    if (LaseredSensorValue > (SensorValue + 10)){
      response = 2;
      }
    else {
      response = 1;
      }
    command = 0; //Resetting Command to Blank command after completion
    }

   // Laser Calibration
   else if (command == 52){
    SensorValue = analogRead(sensorPin); // read the value from the sensor
    digitalWrite(laser_trigger_pin, HIGH);
    delay(1000);
    LaseredSensorValue = analogRead(sensorPin); // read the value from the sensor
    digitalWrite(laser_trigger_pin, LOW);
    if (LaseredSensorValue > (SensorValue + 20)){
      response = 2;
      }
    else {
      response = 1;
      }
    command = 0; //Resetting Command to Blank command after completion
    }

   // Laser On
   else if (command == 53){
    digitalWrite(laser_trigger_pin, HIGH);
    response = 1;
    command = 0; //Resetting Command to Blank command after completion
    }

   // Laser Off
   else if (command == 54){
    digitalWrite(laser_trigger_pin, LOW);
    response = 1;
    command = 0; //Resetting Command to Blank command after completion
    }



    
   }

// Homing motor command Stepper A
//void stepperA_home_forward_command(int Limit_switch){
//  while (digitalRead(Limit_switch) == HIGH && digitalRead(Time_out_pin) == HIGH) { // if it touches an end-stop 
//  myStepperA.setMaxSpeed(200);
//  myStepperA.setSpeed(100); 
//  myStepperA.runSpeed(); 
//  }
//}

// Homing motor command reverse A
void stepperA_home_forward_command(int Limit_switch){
  while (digitalRead(Limit_switch) == HIGH && digitalRead(Time_out_pin) == HIGH) { // if it touches an end-stop 
  myStepperA.setMaxSpeed(200);
  myStepperA.setSpeed(100); 
  myStepperA.runSpeed(); 
  }
}

// Homing motor command reverse A
void stepperA_home_reverse_command(int Limit_switch){
  while (digitalRead(Limit_switch) == HIGH && digitalRead(Time_out_pin) == HIGH) { // if it touches an end-stop 
  myStepperA.setMaxSpeed(200);
  myStepperA.setSpeed(-100); 
  myStepperA.runSpeed(); 
  }
}

// Homing motor command reverse A no timeout
void stepperA_home_forward_command_no_timeout(int Limit_switch){
  int stepper_counter = 0;
  myStepperA.setCurrentPosition(0);
  myStepperA.moveTo(550);
  
  while (myStepperA.distanceToGo() > 1 && digitalRead(Limit_switch) == HIGH) { // if it touches an end-stop
    Serial.println(myStepperA.distanceToGo());
    myStepperA.setMaxSpeed(1000);
    myStepperA.setSpeed(200);  
    myStepperA.runSpeed();
  }

  if (digitalRead(Limit_switch) == HIGH) {
    stepperA_home_reverse_command_no_timeout(LS_right_rfid);
    Serial.println("Limit Switch not reached");
    }


  
//  while (digitalRead(Limit_switch) == HIGH && stepper_counter <= 600) { // if it touches an end-stop 
//  stepper_counter += 1;
////  myStepperA.setMaxSpeed(200);
////  myStepperA.setSpeed(100); 
////  myStepperA.runSpeed();
//  myStepperA.setCurrentPosition(0); //Reseting Current Position back to zero
//  myStepperA.setMaxSpeed(800);
//  myStepperA.setAcceleration(2000);
//  myStepperA.runToNewPosition(1);
//  }
//  if (stepper_counter >= 100) {
//    stepperA_home_reverse_command_no_timeout(LS_right_rfid);
//    Serial.println("Stepper counter greater than 30");
//    }
}

// Homing motor command reverse A no timeout
void stepperA_home_reverse_command_no_timeout(int Limit_switch){
  int stepper_counter = 0;
  //stepper_counter += 1;
  myStepperA.setCurrentPosition(0);
  myStepperA.moveTo(-720);
  
  while (myStepperA.distanceToGo() < -1 && digitalRead(Limit_switch) == HIGH) { // if it touches an end-stop
    Serial.println(myStepperA.distanceToGo());
    myStepperA.setMaxSpeed(1000);
    myStepperA.setSpeed(-200);  
    myStepperA.runSpeed();
  }
//  myStepperA.setCurrentPosition(0); //Reseting Current Position back to zero
//  myStepperA.setMaxSpeed(800);
//  myStepperA.setAcceleration(2000);
//  myStepperA.runToNewPosition(-10);
  
  if (digitalRead(Limit_switch) == HIGH) {
    stepperA_home_forward_command_no_timeout(LS_left_rfid);
    Serial.println("Limit Switch not reached");
    }
}


// Homing motor command Stepper B 
void stepperB_home_forward_command(int Limit_switch){
  while (digitalRead(Limit_switch) == HIGH && digitalRead(Time_out_pin) == HIGH) { // if it touches an end-stop 
  myStepperB.setMaxSpeed(200);
  myStepperB.setSpeed(100); 
  myStepperB.runSpeed(); 
  }
}

// Homing motor command Stepper B with timeout trigger
void stepperB_home_forward_command_timeout(int Limit_switch){
  while (digitalRead(Limit_switch) == HIGH && digitalRead(Time_out_pin) == HIGH) { // if it touches an end-stop 
  myStepperB.setMaxSpeed(200);
  myStepperB.setSpeed(100); 
  myStepperB.runSpeed(); 
  }
}

// Homing motor command reverse B
void stepperB_home_reverse_command(int Limit_switch){
  while (digitalRead(Limit_switch) == HIGH && digitalRead(Time_out_pin) == HIGH) { // if it touches an end-stop 
  myStepperB.setMaxSpeed(200);
  myStepperB.setSpeed(-100); 
  myStepperB.runSpeed(); 
  }
}

// Homing motor command 
void stepperC_home_forward_command(int Limit_switch){
  while (digitalRead(Limit_switch) == HIGH && digitalRead(Time_out_pin) == HIGH) { // if it touches an end-stop 
  myStepperC.setMaxSpeed(200);
  myStepperC.setSpeed(100); 
  myStepperC.runSpeed(); 
  }
}

// Homing motor command reverse
void stepperC_home_reverse_command(int Limit_switch){
  while (digitalRead(Limit_switch) == HIGH && digitalRead(Time_out_pin) == HIGH) { // if it touches an end-stop 
  myStepperC.setMaxSpeed(200);
  myStepperC.setSpeed(-100); 
  myStepperC.runSpeed(); 
  }
}

// Standard Motor Running Sequence
void run_motor() {
  // Spin motor normal
  digitalWrite(dirPin, step_direction);
  for(int x = 0; x < total_steps; x++)
  {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(steps_delay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(steps_delay);
  }
}
