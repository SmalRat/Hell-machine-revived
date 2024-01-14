#include "Arduino.h"
#include <ps5Controller.h>
#include <CAN.h>

void setup()
{
    Serial.begin(921600);
    ps5.begin("4c:b9:9b:4c:5b:9a");
    Serial.println("Ready.");
    if (ps5.isConnected() == true) {
      Serial.println("PS5 controller found");
    }
    else {
      Serial.println("PS5 controller not found");
    }
    if (!CAN.begin(1E6)) {
      Serial.println("Starting CAN failed!");
    }
}

void loop() {
//  while (ps5.isConnected() == false) { // commented out as ps5 controller seems to connect quicker when microcontroller is doing nothing
//    Serial.println("PS5 controller not found");
//    delay(300);
//  }

  while (ps5.isConnected() == true) {
    uint8_t control_buttons=0;

    if (ps5.Right()) {Serial.println("Right Button"); control_buttons=control_buttons | (1<<0);}
    if (ps5.Down()) {Serial.println("Down Button"); control_buttons=control_buttons | (1<<1);}
    if (ps5.Up()) {Serial.println("Up Button"); control_buttons=control_buttons | (1<<2);}
    if (ps5.Left()) {Serial.println("Left Button"); control_buttons=control_buttons | (1<<3);}

    if (ps5.Square()) {Serial.println("Square Button"); control_buttons=control_buttons | (1<<4);}
    if (ps5.Cross()) {Serial.println("Cross Button"); control_buttons=control_buttons | (1<<5);}
    if (ps5.Circle()) {Serial.println("Circle Button"); control_buttons=control_buttons | (1<<6);}
    if (ps5.Triangle()) {Serial.println("Triangle Button"); control_buttons=control_buttons | (1<<7);}

    // if (ps5.UpRight()) Serial.println("Up Right");
    // if (ps5.DownRight()) Serial.println("Down Right");
    // if (ps5.UpLeft()) Serial.println("Up Left");
    // if (ps5.DownLeft()) Serial.println("Down Left");

    uint8_t other_buttons=0;

    if (ps5.L1()) {Serial.println("L1 Button"); other_buttons=other_buttons | (1<<0);}
    if (ps5.R1()) {Serial.println("R1 Button"); other_buttons=other_buttons | (1<<1);}
    

    if (ps5.Share()) {Serial.println("Share Button"); other_buttons=other_buttons | (1<<2);}
    if (ps5.Options()) {Serial.println("Options Button"); other_buttons=other_buttons | (1<<3);}
    if (ps5.L3()) {Serial.println("L3 Button"); other_buttons=other_buttons | (1<<4);}
    if (ps5.R3()) {Serial.println("R3 Button"); other_buttons=other_buttons | (1<<5);}
    if (ps5.PSButton()) {Serial.println("PS Button"); other_buttons=other_buttons | (1<<6);}
    if (ps5.Touchpad()) {Serial.println("Touch Pad Button"); other_buttons=other_buttons | (1<<7);}



    // if (ps5.L1()) Serial.println("L1 Button");
    // if (ps5.R1()) Serial.println("R1 Button");

    // if (ps5.Share()) Serial.println("Share Button");
    // if (ps5.Options()) Serial.println("Options Button");
    // if (ps5.L3()) Serial.println("L3 Button");
    // if (ps5.R3()) Serial.println("R3 Button");
    // if (ps5.PSButton()) Serial.println("PS Button");
    // if (ps5.Touchpad()) Serial.println("Touch Pad Button");



    if (ps5.L2()) {
      Serial.printf("L2 button at %d\n", ps5.L2Value());
    }
    if (ps5.R2()) {
      Serial.printf("R2 button at %d\n", ps5.R2Value());
    }

    if (ps5.LStickX()) {
      Serial.printf("Left Stick x at %d\n", ps5.LStickX());
    }
    if (ps5.LStickY()) {
      Serial.printf("Left Stick y at %d\n", ps5.LStickY());
    }
    if (ps5.RStickX()) {
      Serial.printf("Right Stick x at %d\n", ps5.RStickX());
    }
    if (ps5.RStickY()) {
      Serial.printf("Right Stick y at %d\n", ps5.RStickY());
    }

    // if (ps5.Charging()) Serial.println("The controller is charging"); //doesn't work
    // if (ps5.Audio()) Serial.println("The controller has headphones attached"); //doesn't work
    // if (ps5.Mic()) Serial.println("The controller has a mic attached"); //doesn't work

    Serial.printf("Battery Level : %d\n", ps5.Battery()); //doesn't work

    Serial.println();

    CAN.beginPacket(0x12);
    CAN.write(ps5.LStickX());
    CAN.write(ps5.LStickY());
    CAN.write(ps5.RStickX());
    CAN.write(ps5.RStickY());
    CAN.write(ps5.L2());
    CAN.write(ps5.R2());
    CAN.write(control_buttons);
    CAN.write(other_buttons);
    CAN.endPacket();
    // This delay is to make the output more human readable
    // Remove it when you're not trying to see the output
    delay(300);
  }
}
