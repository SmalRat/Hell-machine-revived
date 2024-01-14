// #include <Arduino.h>

// unsigned long lastflash;
// int RPM;
// void ICACHE_RAM_ATTR sens() {
//   RPM++;
// }

// void setup() {
//     Serial.begin(115200);
//     pinMode(2, INPUT_PULLUP); 
//     attachInterrupt(digitalPinToInterrupt(2), sens, RISING);
//     //SENSOR: GPIO0 (NodeMCU - D3)
// }

// void loop() {
//   noInterrupts();
//   int wings= 3;  // no of wings of rotating object, for disc object use 1 with white tape on one side
//   int RPMnew = RPM/wings;  //here we used fan which has 3 wings
//   Serial.print(RPMnew);
//   Serial.print(" Rot/sec :");  //Revolutions per second
//   Serial.print((RPMnew*60));
//   Serial.println("Rot/min. ");   //Revolutions per minute
//   RPM=0;
//   interrupts(); 
//   delay(1000);  //1 second.
// }