/*
 * Blink
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */

#include <Arduino.h>

// Set LED_BUILTIN if it is not defined by Arduino framework
#ifndef LED_BUILTIN
    #define LED_BUILTIN 2
#endif

int count1 = 0;
int count2 = 0;

void task1(void *mock){
    for (;;){
        Serial.print("Counter 1: ");
        Serial.println(count1++);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void task2(void *mock){
    for (;;){
        Serial.print("Counter 2: ");
        Serial.println(count2++);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Hello, world!");
  
  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  xTaskCreate(
    task1, 
    "Task 1",
    512,
    NULL,
    1,
    NULL
  );

  xTaskCreate(
    task2, 
    "Task 2",
    512,
    NULL,
    1,
    NULL
  );
}


void loop()
{
  // turn the LED on (HIGH is the voltage level)
  digitalWrite(LED_BUILTIN, HIGH);
  // wait for a second
  delay(1000);
  // turn the LED off by making the voltage LOW
  digitalWrite(LED_BUILTIN, LOW);
   // wait for a second
  delay(1000);
}