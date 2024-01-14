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

#define SPEED_READ_PIN_1 34
#define SPEED_READ_PIN_2 35

#define SLIDE_WINDOW_IN_MILLIS 1000
#define BUFFER_SIZE_IN_SECS (SLIDE_WINDOW_IN_MILLIS / MILLIS_IN_SEC)
#define STANDARD_BUFFER_SIZE 1000
#define MEASUREMENT_PERIOD 1
#define MILLIS_IN_SEC 1000
#define MEASUREMENT_PER_SEC (MILLIS_IN_SEC / MEASUREMENT_PERIOD)

#define BUFFER_SIZE static_cast<int>(BUFFER_SIZE_IN_SECS * MEASUREMENT_PER_SEC)

#define COUNTER_MULTIPLIER STANDARD_BUFFER_SIZE / BUFFER_SIZE


static bool buffer_wheel_1[BUFFER_SIZE];
static bool buffer_wheel_2[BUFFER_SIZE];
static bool buffer_wheel_3[BUFFER_SIZE];
static bool buffer_wheel_4[BUFFER_SIZE];

static bool cur_state_wheel_1 = false;
static bool cur_state_wheel_2 = false;
static bool cur_state_wheel_3 = false;
static bool cur_state_wheel_4 = false;

static bool prev_state_wheel_1 = false;
static bool prev_state_wheel_2 = false;
static bool prev_state_wheel_3 = false;
static bool prev_state_wheel_4 = false;

static size_t current_tick_wheel_1 = 0;
static size_t current_tick_wheel_2 = 0;
static size_t current_tick_wheel_3 = 0;
static size_t current_tick_wheel_4 = 0;
//static size_t changes = 0;

static long long counter_1 = 0;
static long long counter_2 = 0;
static long long counter_3 = 0;
static long long counter_4 = 0;

inline void measure_speed(bool &cur_state, bool &prev_state, bool *buffer, long long &counter, size_t &current_tick, int pin){
  cur_state = digitalRead(pin);
  if (cur_state != prev_state){
    if (!buffer[current_tick]){
      counter++;
    }
    buffer[current_tick] = true;
  }
  else{
    if (buffer[current_tick]){
      counter--;
    }
    buffer[current_tick] = false;
  }
  prev_state = cur_state;
  current_tick++;
  current_tick %= BUFFER_SIZE;
}

void task_measure_wheel_speed(void *mock){
    for (;;){
      measure_speed(cur_state_wheel_1, prev_state_wheel_1, buffer_wheel_1, counter_1, current_tick_wheel_1, SPEED_READ_PIN_1);
      measure_speed(cur_state_wheel_2, prev_state_wheel_2, buffer_wheel_2, counter_2, current_tick_wheel_2, SPEED_READ_PIN_2);
      measure_speed(cur_state_wheel_3, prev_state_wheel_3, buffer_wheel_3, counter_3, current_tick_wheel_3, SPEED_READ_PIN_2);
      measure_speed(cur_state_wheel_4, prev_state_wheel_4, buffer_wheel_4, counter_4, current_tick_wheel_4, SPEED_READ_PIN_2);
      vTaskDelay(MEASUREMENT_PERIOD / portTICK_PERIOD_MS);
    }
}

// void task_measure_wheel_2_speed(void *mock){
//     for (;;){
//       measure_speed(cur_state_wheel_2, prev_state_wheel_2, buffer_wheel_2, counter_2, current_tick_wheel_2);
//       vTaskDelay(MEASUREMENT_PERIOD / portTICK_PERIOD_MS);
//     }
// }

// void task_measure_wheel_3_speed(void *mock){
//     for (;;){
//       measure_speed(cur_state_wheel_2, prev_state_wheel_2, buffer_wheel_2, counter_2, current_tick_wheel_2);
//       vTaskDelay(MEASUREMENT_PERIOD / portTICK_PERIOD_MS);
//     }
// }

// void task_measure_wheel_4_speed(void *mock){
//     for (;;){
//       measure_speed(cur_state_wheel_2, prev_state_wheel_2, buffer_wheel_2, counter_2, current_tick_wheel_2);
//       vTaskDelay(MEASUREMENT_PERIOD / portTICK_PERIOD_MS);
//     }
// }

void task_print_speeds(void *mock){
    for (;;){
        Serial.print("Speed 1: ");
        Serial.println(counter_1);
        Serial.print("Speed 2: ");
        Serial.println(counter_2);
        // Serial.print("Speed 3: ");
        // Serial.println(counter_3);
        // Serial.print("Speed 4: ");
        // Serial.println(counter_4);
        //Serial.println(changes);
        //Serial.println(digitalRead(SPEED_READ_PIN));
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}


// void task3(void *mock){
//   prev_state = digitalRead(SPEED_READ_PIN);
//   bool s = false;
//     for (;;){
//         cur_state = digitalRead(SPEED_READ_PIN);
//         if (cur_state != prev_state && !s){
//           Serial.println(cur_state);
//           s = true;
//         }
//         if (cur_state != s){
//           s = false;
//           Serial.println(cur_state);
//         }
//         vTaskDelay(1 / portTICK_PERIOD_MS);
//     }
// }


void setup()
{
  Serial.begin(115200);
  Serial.println("Hello, world!");
  Serial.println(MEASUREMENT_PERIOD / portTICK_PERIOD_MS);
  
  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(SPEED_READ_PIN_1, INPUT);
  pinMode(SPEED_READ_PIN_2, INPUT);

  prev_state_wheel_1 = digitalRead(SPEED_READ_PIN_1);
  prev_state_wheel_2 = digitalRead(SPEED_READ_PIN_2);
  prev_state_wheel_3 = digitalRead(SPEED_READ_PIN_2);
  prev_state_wheel_4 = digitalRead(SPEED_READ_PIN_2);

  for (size_t i = 0; i < BUFFER_SIZE; ++i){
    buffer_wheel_1[i] = false;
    buffer_wheel_2[i] = false;
    buffer_wheel_3[i] = false;
    buffer_wheel_4[i] = false;
  }

  // for (size_t i = 0; i < 100000; ++i){
  //   Serial.println(esp_timer_get_time());
  //   vTaskDelay(100);
  // }

  xTaskCreate(
    task_measure_wheel_speed, 
    "Measure speeds task",
    2048,
    NULL,
    1,
    NULL
  );

  xTaskCreate(
    task_print_speeds, 
    "Print speeds task",
    2048,
    NULL,
    1,
    NULL
  );

  // xTaskCreate(
  //   task3, 
  //   "Task 3",
  //   1024,
  //   NULL,
  //   1,
  //   NULL
  // );
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