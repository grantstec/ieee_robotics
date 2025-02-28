#include <Arduino.h>
#include <TMCStepper.h>
#include <AccelStepper.h>

// TMC2209 Configuration
#define L_SERIAL Serial3  // Left motor
#define R_SERIAL Serial2  // Right motor
#define R_SENSE 0.11f

TMC2209Stepper driverL(&L_SERIAL, R_SENSE, 0x00);  // Replace 0x00 with the actual driver address
TMC2209Stepper driverR(&R_SERIAL, R_SENSE, 0x00);  // Replace 0x01 with the actual driver address

// Stepper Pins
#define R_STEP_PIN 54
#define R_DIR_PIN 55
#define R_ENABLE_PIN 38

#define L_STEP_PIN 26
#define L_DIR_PIN 28
#define L_ENABLE_PIN 24

// Stepper Objects
AccelStepper stepperL(AccelStepper::DRIVER, L_STEP_PIN, L_DIR_PIN);
AccelStepper stepperR(AccelStepper::DRIVER, R_STEP_PIN, R_DIR_PIN);

// Constants
const unsigned long REPORT_INTERVAL = 100;
unsigned long last_report = 0;

void setup() {
  Serial.begin(115200);
  
  // TMC2209 Initialization
  L_SERIAL.begin(115200);
  R_SERIAL.begin(115200);
  
  driverL.begin();
  driverL.pdn_disable(true);
  driverL.toff(5);
  driverL.en_spreadCycle(false);
  driverL.microsteps(16);
  driverL.irun(31);
  driverL.ihold(5);

  driverR.begin();
  driverR.pdn_disable(true);
  driverR.toff(5);
  driverR.en_spreadCycle(false);
  driverR.microsteps(16);
  driverR.irun(31);  // Set to maximum current value
  driverR.ihold(5);

  // Stepper Configuration
  stepperL.setEnablePin(L_ENABLE_PIN);
  stepperL.setPinsInverted(true, false, true);
  stepperL.enableOutputs();
  stepperL.setMaxSpeed(40000);
  stepperL.setAcceleration(30000);

  stepperR.setEnablePin(R_ENABLE_PIN);
  stepperR.setPinsInverted(false, false, true);
  stepperR.enableOutputs();
  stepperR.setMaxSpeed(40000);
  stepperR.setAcceleration(30000);
}

void loop() {
  // Handle incoming commands
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd.startsWith("CMD:")) {
      cmd.remove(0, 4);
      int comma = cmd.indexOf(',');
      if (comma != -1) {
        long left = cmd.substring(0, comma).toInt();
        long right = cmd.substring(comma+1).toInt();
        
        stepperL.move(left);
        stepperR.move(right);
      }
    }
  }

  // Run motors
  stepperL.run();
  stepperR.run();

  // Report positions
  if (millis() - last_report > REPORT_INTERVAL) {
    Serial.print("STEPS:");
    Serial.print(stepperL.currentPosition());
    Serial.print(",");
    Serial.println(stepperR.currentPosition());
    last_report = millis();
  }
}
