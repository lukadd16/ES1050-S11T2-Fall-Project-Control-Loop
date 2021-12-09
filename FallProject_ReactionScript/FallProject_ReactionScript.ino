// Import required libraries
#include <Wire.h>
#include "Adafruit_MPRLS.h"
#include "Plotter.h"

// Initialize class used to send data over serial that can be plotted
Plotter p;

// Initialize class used to interface with the pressure sensor
#define RESET_PIN  -1  // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN    -1  // set to any GPIO pin to read end-of-conversion by pin
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);

/*
 * Define the pins that control each solenoid valve.
 * SOLENOID_OUT --> the valve letting pressure out
 * SOLENOID_IN --> the valve letting vinegar in
 */
#define SOLENOID_OUT 12
#define SOLENOID_IN  11

/*
 * Datatype that outlines three possible "states" to describe the pressure within the reaction vessel.
 * BELOW = rate of change of pressure is smaller than is required to reach the setpoint (stop releasing pressure & start adding vinegar)
 * GOOD = rate of change of pressure matches the rate required to reach the setpoint (no action required)
 * ABOVE = rate of change of pressure is larger than is required to reach the setpoint (stop adding vinegar & start releasing pressure)
 */
enum PState {
  BELOW,
  GOOD,
  ABOVE,
  UNKNOWN
};

/* Declare variables to be used later */
float setpoint = 10; // The pressure (in Psi) that we wish to reach by the end of the evaluation period (three minutes)
float prevPressure = 0, currPressure = 0; // Keep track of pressure readings from the reaction vessel (in Psi)
float currSlope; // The rate of change of pressure between the previous and current pressure readings
float targetSlope; // The rate of change of pressure needed to reach the setpoint from the current pressure
enum PState currState;
float atmosphereCorrectionFactor;
int interval = 1000; // Time (in miliseconds) between each subsequent call to performAction() (ideally)
unsigned long prevMillis = 0UL; // The last time in milliseconds (since the script started) that performAction() was called
unsigned long currMillis; // ...
// int currSeconds = (currMillis / 1000); // KEEP GETTING "cannot bind non-const lvalue reference..." ERROR AND DON'T KNOW HOW TO FIX IT, I GUESS MILLISECONDS WILL HAVE TO DO
unsigned long evaluationMillis = 180000UL; // The evaluation period spans three minutes

/**
 * This function will only get called once, every time the Arduino executes this script.
 */
void setup() {
  // To control each solenoid valve, we want to output voltage from its previously defined board pin.
  // Note: HIGH = valve open, LOW = valve closed
  pinMode(SOLENOID_OUT, OUTPUT);
  pinMode(SOLENOID_IN, OUTPUT);

  // Ensure we can communicate with the pressure sensor
  Serial.begin(115200);
  Serial.println("MPRLS Sensor Interface Test");
  if (! mpr.begin()) {
    Serial.println("Failed to communicate with MPRLS sensor, check wiring?");
    while (1) {
      delay(10);
    }
  }
  Serial.println("Found MPRLS sensor, proceeding...");

  // Find the atmospheric pressure of the room/within the reaction vessel prior to the reaction starting
  atmosphereCorrectionFactor = getRawPressurePsi();

  // Setup Plotter
  p.Begin();
  p.AddXYGraph("Pressure vs. Time", 18000, "Time (ms)", currMillis, "Pressure (Psi)", currPressure);
  p.AddXYGraph("Slope vs. Time", 18000, "Time (ms)", currMillis, "ROC of Pressure (Psi/~sec)", currSlope);
  p.SetColor(0, "red");
  p.SetColor(1, "orange");
}

/**
 * This function will repeatedly get called after setup() has finished.
 */
void loop() {
  // Take a pressure reading
  prevPressure = currPressure;
//  currPressure = getRawPressurePsi(); // NOTE: if using ACF, remember to replace this function call with getAdjustedPressurePsi();
  currPressure = getAdjustedPressurePsi();

  // The amount of time (in milliseconds) it has been since the script began running
  currMillis = millis();
  
  // Corrective action on the reaction will only be taken if a sufficient amount of time has passed since the last action was taken.
  if (currMillis - prevMillis >= interval) {
    compute();

    performAction();

    prevMillis = currMillis;
  }

//  // Send the current pressure and slope over serial so that a computer can be used to plot the reaction as it progresses
//  Serial.print("Pressure (Psi): "); Serial.println(pressure_Psi);

  // Plot all current data on the attached computer
  p.Plot(); // TODO: perhaps plot only when we also update the slope?
}

/**
 * This function takes a pressure reading from the sensor and converts it to Psi.
 */
float getRawPressurePsi() {
  float pressure_hPa = mpr.readPressure();
  return (pressure_hPa / 68.947572932);
}

/**
 * Modified version of getRawPressurePsi() that accounts for atmospheric pressure of the room/within the device prior to the reaction starting.
 */
float getAdjustedPressurePsi() {
  float pressure_Psi = getRawPressurePsi();
  return (pressure_Psi - atmosphereCorrectionFactor);
}

/**
 * This function determines which corrective action to take in order to reach the setpoint in a linear fashion.
 */
void compute() {
  float acceptedRange = 20; // TODO: can be tweaked after seeing how agressive the effect each corrective action has on the pressure.
  
  // Calculate the current rate of change of pressure (as an average)
  currSlope = ((currPressure - prevPressure) / (currMillis - prevMillis));

  // Calculate the target rate of change of pressure (as an average)
  targetSlope = ((setpoint - currPressure) / (evaluationMillis - currMillis));

  // Compare the actual vs. desired slopes
  if (currSlope < targetSlope) {
    currState = BELOW;
  } else if (isWithinPercentage(currSlope, targetSlope, acceptedRange)) { // Realistically, the two slopes will never be equal, so instead we check if they are equal within a certain percentage of each other
    currState = GOOD;
  } else if (currSlope > targetSlope) {
    currState = ABOVE;
  } else {
    currState = UNKNOWN;
  }  
}

/**
 * This function modifies the state of each valve to achieve the desired outcome needed to correct the current pressure state within the reaction vessel.
 */
void performAction() {
  switch (currState) {
    case BELOW:
      // Pressure slope too low, correct by increasing pressure within the vessel.
      digitalWrite(SOLENOID_OUT, LOW); // Close valve to keep pressure in
      digitalWrite(SOLENOID_IN, HIGH); // Open valve to let vinegar in
      break;
    case ABOVE:
      // Pressure slope too high, correct by decreasing pressure within the vessel.
      digitalWrite(SOLENOID_OUT, HIGH); // Open valve to let pressure out
      digitalWrite(SOLENOID_IN, LOW); // Close valve to prevent additional vinegar from reacting
      break;
    default:
      // No corrective action needed
      digitalWrite(SOLENOID_OUT, LOW); // Close valve to keep pressure in
      digitalWrite(SOLENOID_IN, LOW); // Close valve to prevent additional vinegar from reacting
      break;
  }  
}

/**
 * Helper function; performs a percent difference test on two values and checks if the difference falls within the accepted range.
 */
bool isWithinPercentage(float numberA, float numberB, float range) {
  float percentage = 0;
  percentage = ((numberA - numberB) / ((numberA + numberB) / 2)) * 100;
  percentage = abs(percentage);

  return (percentage <= range);
}
