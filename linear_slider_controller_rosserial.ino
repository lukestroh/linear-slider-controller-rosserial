/*
 * Linear slider controller
 * Author: Luke Strohbehn
 * April 10, 2023
 * 
 * 
 * The structure of this script is based on two normally-closed limit switches, configured 
 * with the onboard input resistors using INPUT_PULLUP. Adjust the logic according to your
 * hardware logic in limitswitch.cpp
 * 
 * 
 * 
 */

#include "limitswitch.h"
#include "rosnode.h"
#include <AccelStepper.h>
#include <EEPROM.h>
#include <stdio.h>

#define DEBUG true
 
// Limit switch pins
const uint8_t limit_switch0 {2};
const uint8_t limit_switch1 {3};

// Stepper pins
const uint8_t spin0 {4};
const uint8_t spin1 {5};
const uint8_t spin2 {6};

// EEPROM position address
int EEPROM_pos_addr {0};
int EEPROM_write_counter;
int EEPROM_write_counter_addr;

// Desired stepper position
int stepper_target;

// Switches
LimitSwitch ls0(limit_switch0);
LimitSwitch ls1(limit_switch1);

// Stepper
AccelStepper stepper(spin0, spin1, spin2);

// Node handler
ros::NodeHandle nh;

// ROS node
ROSNode rn(nh);


void update_stepper(int pos) {
    /* Increment stepper position for event loop */
    stepper.moveTo(pos); // moveTo will set target to an absolute position, runs at the last set speed
    if (stepper.distanceToGo() > 0) {
        stepper.run();
        Serial.println(stepper.currentPosition());
    } else {
        stepper.disableOutputs();
        // Serial.println(F("Done."));
        // EEPROM write takes 3.3ms, can only be written 100,000 times. Only use for when position finishes updating? using `update()` only writes if the value is different
        // EEPROM.update(EEPROM_pos_addr, stepper.currentPosition)
    }
}

void calibrate_stepper() {
    /* Zeros stepper position to limit_switch0 */

}

/*******************************************

    Main

********************************************/

void setup() {
    Serial.begin(115200);

    while (!Serial){;}

    stepper.setMaxSpeed(5000);
    stepper.setAcceleration(1000);
    stepper.disableOutputs(); // re-enable pins with enableOutputs();
    stepper.setCurrentPosition(0); // to be used in calibration step. Store last position in eeprom?

    // Init node handler
    nh.initNode();
//    nh.advertise(rn.publisher);

    rn.connect();
}

void loop() {

    // Return current stepper position
    // Will need up update to physical dimensions???
    int pos = stepper.currentPosition();

    rn.msg_f32.data = 3.14159;
    rn.publisher.publish(rn.msg_f32);

    // Detect interrupts  /////////////////// NOTE*********** does not stop stepper in loop, need to set another flag
    if (ls0.read_switch()) {
        Serial.println(F("Interrupt 0 triggered"));
        stepper.stop();
        stepper.runToPosition();
        stepper_target = stepper.currentPosition();
    }
    if (ls1.read_switch()) {
        Serial.println(F("Interrupt 1 triggered"));
        stepper.stop();
        stepper.runToPosition();
        stepper_target = stepper.currentPosition();
    }


    update_stepper(stepper_target); // Custom stepper class, holds target position and always tries to move there?

}
