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
#include "EthTCP.h"
#include <AccelStepper.h>
#include <EEPROM.h>
#include <stdio.h>

#define DEBUG 0

 
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
float stepper_target;

// Switches
LimitSwitch ls0(limit_switch0);
LimitSwitch ls1(limit_switch1);

// Stepper
AccelStepper stepper(spin0, spin1, spin2);

// Ethernet
Eth eth0;
bool _connected = false;


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

    _connected = eth0.connect();
    
}



void loop() {
    // reconnect if broken, and maintain DHCP lease
    _connected = eth0.connect();

    // Convert float to char array for sending over TCP
    char _buffer[13];
    float f = 3.1415926;
    dtostrf(f, 10, 10, _buffer);

    eth0.send_data(_buffer);
    
    if (_connected) {
        // Read ethernet data buffer
        eth0.read_data();
        if (eth0.newData) {

#if DEBUG
        Serial.print(F("Received command: "));
        Serial.println(eth0.receivedChars);
#endif // DEBUG

            // convert char array to float
//            sscanf(eth0.receivedChars, "%f", &stepper_target);
            stepper_target = atof(eth0.receivedChars);
#if DEBUG
            Serial.print(F("stepper_target: "));
            Serial.println(stepper_target, 5); // digits of precision printed out
#endif // DEBUG
            eth0.newData = false;
        }

        // Return current stepper position
        // Will need up update to physical dimensions???
//        eth0.send_data(stepper.currentPosition());

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

#if DEBUG
//    delay(500);
#endif // DEBUG

}
