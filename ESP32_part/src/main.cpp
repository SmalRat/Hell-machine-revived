#include <esp32-hal-timer.h>
#include <HardwareSerial.h>

#define SPEED_READ_PIN_1 34
#define SPEED_READ_PIN_2 35

#define SLIDE_WINDOW_IN_MILLIS 1000
#define BUFFER_SIZE_IN_SECS (SLIDE_WINDOW_IN_MILLIS / MILLIS_IN_SEC)
#define STANDARD_BUFFER_SIZE 1000
#define MEASUREMENT_PERIOD 0.1
#define MILLIS_IN_SEC 1000
#define MEASUREMENT_PER_SEC (MILLIS_IN_SEC / MEASUREMENT_PERIOD)

#define BUFFER_SIZE static_cast<int>(BUFFER_SIZE_IN_SECS * MEASUREMENT_PER_SEC)

#define COUNTER_MULTIPLIER STANDARD_BUFFER_SIZE / BUFFER_SIZE


hw_timer_t *wheel_timer_1 = NULL;
hw_timer_t *wheel_timer_2 = NULL;
hw_timer_t *wheel_timer_3 = NULL;
hw_timer_t *wheel_timer_4 = NULL;


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

static long long counter_1 = 0;
static long long counter_2 = 0;
static long long counter_3 = 0;
static long long counter_4 = 0;



void IRAM_ATTR wheel_1_speed_measure(){
  cur_state_wheel_1 = digitalRead(SPEED_READ_PIN_1);
  if (cur_state_wheel_1 != prev_state_wheel_1){
    if (!buffer_wheel_1[current_tick_wheel_1]){
      counter_1++;
    }
    buffer_wheel_1[current_tick_wheel_1] = true;
  }
  else{
    if (buffer_wheel_1[current_tick_wheel_1]){
      counter_1--;
    }
    buffer_wheel_1[current_tick_wheel_1] = false;
  }
  prev_state_wheel_1 = cur_state_wheel_1;
  current_tick_wheel_1++;
  current_tick_wheel_1 %= BUFFER_SIZE;
}

void IRAM_ATTR wheel_2_speed_measure(){
  cur_state_wheel_2 = digitalRead(SPEED_READ_PIN_2);
  if (cur_state_wheel_2 != prev_state_wheel_2){
    if (!buffer_wheel_2[current_tick_wheel_2]){
      counter_2++;
    }
    buffer_wheel_2[current_tick_wheel_2] = true;
  }
  else{
    if (buffer_wheel_2[current_tick_wheel_2]){
      counter_2--;
    }
    buffer_wheel_2[current_tick_wheel_2] = false;
  }
  prev_state_wheel_2 = cur_state_wheel_2;
  current_tick_wheel_2++;
  current_tick_wheel_2 %= BUFFER_SIZE;
}


void IRAM_ATTR wheel_3_speed_measure(){
  cur_state_wheel_3 = digitalRead(SPEED_READ_PIN_1);
  if (cur_state_wheel_3 != prev_state_wheel_3){
    if (!buffer_wheel_3[current_tick_wheel_3]){
      counter_3++;
    }
    buffer_wheel_3[current_tick_wheel_3] = true;
  }
  else{
    if (buffer_wheel_3[current_tick_wheel_3]){
      counter_3--;
    }
    buffer_wheel_3[current_tick_wheel_3] = false;
  }
  prev_state_wheel_3 = cur_state_wheel_3;
  current_tick_wheel_3++;
  current_tick_wheel_3 %= BUFFER_SIZE;
}


void IRAM_ATTR wheel_4_speed_measure(){
  cur_state_wheel_4 = digitalRead(SPEED_READ_PIN_1);
  if (cur_state_wheel_4 != prev_state_wheel_4){
    if (!buffer_wheel_4[current_tick_wheel_4]){
      counter_4++;
    }
    buffer_wheel_4[current_tick_wheel_4] = true;
  }
  else{
    if (buffer_wheel_4[current_tick_wheel_4]){
      counter_4--;
    }
    buffer_wheel_4[current_tick_wheel_4] = false;
  }
  prev_state_wheel_4 = cur_state_wheel_4;
  current_tick_wheel_4++;
  current_tick_wheel_4 %= BUFFER_SIZE;
}

void setup() {
  Serial.begin(115200);
  
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

  wheel_timer_1 = timerBegin(0, 80, true);
  timerAttachInterrupt(wheel_timer_1, &wheel_1_speed_measure, true);
  timerAlarmWrite(wheel_timer_1, 100, true);

  wheel_timer_2 = timerBegin(1, 80, true);
  timerAttachInterrupt(wheel_timer_2, &wheel_2_speed_measure, true);
  timerAlarmWrite(wheel_timer_2, 100, true);

  wheel_timer_3 = timerBegin(2, 80, true);
  timerAttachInterrupt(wheel_timer_3, &wheel_3_speed_measure, true);
  timerAlarmWrite(wheel_timer_3, 100, true);

  wheel_timer_4 = timerBegin(3, 80, true);
  timerAttachInterrupt(wheel_timer_4, &wheel_4_speed_measure, true);
  timerAlarmWrite(wheel_timer_4, 100, true);

  timerAlarmEnable(wheel_timer_1);
  timerAlarmEnable(wheel_timer_2);
  timerAlarmEnable(wheel_timer_3);
  timerAlarmEnable(wheel_timer_4);
}


void loop() {
  Serial.print("Speed 1: ");
  Serial.println(counter_1);
  Serial.print("Speed 2: ");
  Serial.println(counter_2);
  //Serial.println(current_tick_wheel_1);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}