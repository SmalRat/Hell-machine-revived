#include <esp32-hal-timer.h>
#include <HardwareSerial.h>
#include <BluetoothSerial.h>


BluetoothSerial bluetooth_module;
// Utility constants - start
constexpr size_t CHAR_MIN_SIZE = 0;
constexpr size_t CHAR_MAX_SIZE = 255;

constexpr int MASTER_DEVICE_BAUDRATE = 115200;

// constexpr int V_5=11;
// constexpr int V_5_1=53;
// Utility constants - end

/*
Constants responsible for engine speed and direction control pins - start
*/

constexpr int ENGINE1_BRAKE_PIN = 26;
constexpr int ENGINE2_BRAKE_PIN = 27;
constexpr int ENGINE3_BRAKE_PIN = 32;
constexpr int ENGINE4_BRAKE_PIN = 33;

constexpr int ENGINE1_SPEED_CONTROL_PIN=25;
constexpr int ENGINE2_SPEED_CONTROL_PIN=23;
constexpr int ENGINE3_SPEED_CONTROL_PIN=22;
constexpr int ENGINE4_SPEED_CONTROL_PIN=21;

constexpr int ENGINE1_DIRECTION_CONTROL_PIN=19;
constexpr int ENGINE2_DIRECTION_CONTROL_PIN=18;
constexpr int ENGINE3_DIRECTION_CONTROL_PIN=17;
constexpr int ENGINE4_DIRECTION_CONTROL_PIN=16;

constexpr int TURNING_DIRECTION_PIN_1 = 15;
constexpr int TURNING_DIRECTION_PIN_2 = 14;

constexpr int TURNING_SPEED_PIN_1 = 13;
constexpr int TURNING_SPEED_PIN_2 = 12;
/*
Constants responsible for engine speed and direction control pins - end
*/

// Speed constants - start
constexpr double MAX_SPEED = 255;
constexpr int MOTOR_SPEED_0 = 0;
constexpr int TEST_SPEED = 120; // Out of 255 (?)
constexpr int MOTOR_SPEED_1 = 130;
constexpr int MOTOR_SPEED_2 = 150;
constexpr int MOTOR_SPEED_3 = 170;

constexpr int MAX_SPEED_MODE = 10;
constexpr double SPEED_QUANT = MAX_SPEED / MAX_SPEED_MODE;
// Speed constants - end

// Speed measurement constants and global variables - start
constexpr int ENGINE1_SPEED_READ_PIN = 34;
constexpr int ENGINE2_SPEED_READ_PIN = 35;
constexpr int ENGINE3_SPEED_READ_PIN = 36;
constexpr int ENGINE4_SPEED_READ_PIN = 39;

constexpr size_t MILLIS_IN_SEC = 1000;
constexpr size_t STANDARD_BUFFER_SIZE = 1000;

constexpr size_t SLIDE_WINDOW_IN_MILLIS = 1000;
constexpr size_t BUFFER_SIZE_IN_SECS = (SLIDE_WINDOW_IN_MILLIS / MILLIS_IN_SEC);
constexpr double MEASUREMENT_PERIOD = 0.1;
constexpr double MEASUREMENT_PER_SEC = (MILLIS_IN_SEC / MEASUREMENT_PERIOD);
constexpr size_t BUFFER_SIZE = static_cast<size_t>(BUFFER_SIZE_IN_SECS * MEASUREMENT_PER_SEC);
constexpr double COUNTER_MULTIPLIER = STANDARD_BUFFER_SIZE / BUFFER_SIZE;


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
// Speed measurement constants and global variables - end

// Connection global variables - start
/* 
Initialising max_loops_without_command which contains a maximum number of loops with unavailable bluetooth module.
If it is exceeded - we assume that connection has been lost, so we should stop a car for safety.
*/
int max_loops_without_command = 0;
int loops_without_command = 0;
int total_commands_received = 0;
int time_without_command = 0;

long long int last_received_signal_timestamp = 0; // Another way to ensure we have an established connection.

bool connection_lost = false; // If connection is lost, we stop a car.
// Connection global variables - end

// Other - start
static int SPEED_MAP_TO_COMMAND[256]; 

char current_command = ' '; // Contains a command symbol received with bluetooth.

int current_motor_speed = TEST_SPEED;

constexpr unsigned int SPEED_BASE = 10000;

double TEMP1_DIF;
// Other - end

// Speed measurement ISR - start
void IRAM_ATTR wheel_1_speed_measure(){
  cur_state_wheel_1 = digitalRead(ENGINE1_SPEED_READ_PIN);
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
  cur_state_wheel_2 = digitalRead(ENGINE2_SPEED_READ_PIN);
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
  cur_state_wheel_3 = digitalRead(ENGINE1_SPEED_READ_PIN);
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
  cur_state_wheel_4 = digitalRead(ENGINE1_SPEED_READ_PIN);
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
// Speed measurement ISR - end

void stop();
void no_turn();
void block_wheels();
void unblock_wheels();
void begin_bluetooth_connection();

void setup() {
  for (size_t i = CHAR_MIN_SIZE; i <= CHAR_MAX_SIZE; ++i){
    SPEED_MAP_TO_COMMAND[i] = -1;
  }
  SPEED_MAP_TO_COMMAND['0'] = 0;
  SPEED_MAP_TO_COMMAND['1'] = 1;
  SPEED_MAP_TO_COMMAND['2'] = 2;
  SPEED_MAP_TO_COMMAND['3'] = 3;
  SPEED_MAP_TO_COMMAND['4'] = 4;
  SPEED_MAP_TO_COMMAND['5'] = 5;
  SPEED_MAP_TO_COMMAND['6'] = 6;
  SPEED_MAP_TO_COMMAND['7'] = 7;
  SPEED_MAP_TO_COMMAND['8'] = 8;
  SPEED_MAP_TO_COMMAND['9'] = 9;
  SPEED_MAP_TO_COMMAND['q'] = 10;
  
  // Speed measurement setup - start
  
  
  pinMode(ENGINE1_SPEED_READ_PIN, INPUT);
  pinMode(ENGINE2_SPEED_READ_PIN, INPUT);
  
  prev_state_wheel_1 = digitalRead(ENGINE1_SPEED_READ_PIN);
  prev_state_wheel_2 = digitalRead(ENGINE2_SPEED_READ_PIN);
  prev_state_wheel_3 = digitalRead(ENGINE2_SPEED_READ_PIN);
  prev_state_wheel_4 = digitalRead(ENGINE2_SPEED_READ_PIN);

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
  // Speed measurement setup - end

  // 
  pinMode(ENGINE1_SPEED_CONTROL_PIN, OUTPUT);
  pinMode(ENGINE2_SPEED_CONTROL_PIN, OUTPUT);
  pinMode(ENGINE3_SPEED_CONTROL_PIN, OUTPUT);
  pinMode(ENGINE4_SPEED_CONTROL_PIN, OUTPUT);

  pinMode(ENGINE1_BRAKE_PIN, OUTPUT);
  pinMode(ENGINE2_BRAKE_PIN, OUTPUT);
  pinMode(ENGINE3_BRAKE_PIN, OUTPUT);
  pinMode(ENGINE4_BRAKE_PIN, OUTPUT);

  unblock_wheels();

  pinMode(TURNING_DIRECTION_PIN_1, OUTPUT);
  pinMode(TURNING_DIRECTION_PIN_2, OUTPUT);

  pinMode(TURNING_SPEED_PIN_1, OUTPUT);
  pinMode(TURNING_SPEED_PIN_2, OUTPUT);
  
  // pinMode(V_5, OUTPUT);
  // pinMode(V_5_1, OUTPUT);
  // digitalWrite(V_5, HIGH);
  // digitalWrite(V_5_1, HIGH);

  pinMode(ENGINE1_DIRECTION_CONTROL_PIN, OUTPUT);
  pinMode(ENGINE2_DIRECTION_CONTROL_PIN, OUTPUT);
  pinMode(ENGINE3_DIRECTION_CONTROL_PIN, OUTPUT);
  pinMode(ENGINE4_DIRECTION_CONTROL_PIN, OUTPUT);
  //

  bluetooth_module.begin("Hell Machine");
  Serial.println("123");

  no_turn(); // Don't delete/comment, as wheels start turning immediately on one side. (probably)
  Serial.print("Init!!\n");

  Serial.begin(MASTER_DEVICE_BAUDRATE);
  begin_bluetooth_connection(); // Waits until bluetooth connection is established.
}

void print_task(){
  Serial.print("Speed 1: ");
  Serial.println(counter_1);
  Serial.print("Speed 2: ");
  Serial.println(counter_2);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void no_turn(){
  digitalWrite(TURNING_DIRECTION_PIN_1, LOW);
  digitalWrite(TURNING_DIRECTION_PIN_2, LOW);

  digitalWrite(TURNING_SPEED_PIN_1, LOW);
  digitalWrite(TURNING_SPEED_PIN_2, LOW);
}

void move_forward(){
  analogWrite(ENGINE1_SPEED_CONTROL_PIN, current_motor_speed);
  analogWrite(ENGINE2_SPEED_CONTROL_PIN, current_motor_speed);

  digitalWrite(ENGINE1_DIRECTION_CONTROL_PIN, HIGH);
  digitalWrite(ENGINE2_DIRECTION_CONTROL_PIN, LOW);
  unblock_wheels();
}


void move_backward(){
  analogWrite(ENGINE1_SPEED_CONTROL_PIN, current_motor_speed);
  analogWrite(ENGINE2_SPEED_CONTROL_PIN, current_motor_speed);

  digitalWrite(ENGINE1_DIRECTION_CONTROL_PIN, LOW);
  digitalWrite(ENGINE2_DIRECTION_CONTROL_PIN, HIGH);
  unblock_wheels();
}


void turn_left(){
  digitalWrite(TURNING_DIRECTION_PIN_1, HIGH);
  digitalWrite(TURNING_DIRECTION_PIN_2, HIGH);

  digitalWrite(TURNING_SPEED_PIN_1, LOW);
  digitalWrite(TURNING_SPEED_PIN_2, LOW);
}


void turn_right(){
  digitalWrite(TURNING_DIRECTION_PIN_1, LOW);
  digitalWrite(TURNING_DIRECTION_PIN_2, LOW);

  digitalWrite(TURNING_SPEED_PIN_1, HIGH);
  digitalWrite(TURNING_SPEED_PIN_2, HIGH);
}

void forward_right(){
      turn_right();
      move_forward();
}

    
void backward_right(){
  turn_right();
  move_backward();
}

void forward_left(){
  turn_left();
  move_forward();
}


void backward_left(){
  turn_left();
  move_backward();
}


void stop(){
  analogWrite(ENGINE1_SPEED_CONTROL_PIN, MOTOR_SPEED_0);
  analogWrite(ENGINE2_SPEED_CONTROL_PIN, MOTOR_SPEED_0);
  analogWrite(ENGINE3_SPEED_CONTROL_PIN, MOTOR_SPEED_0);
  analogWrite(ENGINE4_SPEED_CONTROL_PIN, MOTOR_SPEED_0);
  no_turn();
  block_wheels();
}

void nop(){
  analogWrite(ENGINE1_SPEED_CONTROL_PIN, MOTOR_SPEED_0);
  analogWrite(ENGINE2_SPEED_CONTROL_PIN, MOTOR_SPEED_0);
  analogWrite(ENGINE3_SPEED_CONTROL_PIN, MOTOR_SPEED_0);
  analogWrite(ENGINE4_SPEED_CONTROL_PIN, MOTOR_SPEED_0);
  no_turn();
}

void block_wheels(){
  digitalWrite(ENGINE1_BRAKE_PIN, HIGH);
  digitalWrite(ENGINE2_BRAKE_PIN, HIGH);
  digitalWrite(ENGINE3_BRAKE_PIN, HIGH);
  digitalWrite(ENGINE4_BRAKE_PIN, HIGH);
}

void unblock_wheels(){
  digitalWrite(ENGINE1_BRAKE_PIN, LOW);
  digitalWrite(ENGINE2_BRAKE_PIN, LOW);
  digitalWrite(ENGINE3_BRAKE_PIN, LOW);
  digitalWrite(ENGINE4_BRAKE_PIN, LOW);
}

void execute_command(){
  Serial.print(current_command);
  
  switch (current_command)
  {
    case 'F' : {
      move_forward();
      no_turn();
    }
    break;
    case 'B' : {
      move_backward();
      no_turn();
    }
    break;
    case 'L' : {
      turn_left();
    }
    break;
    case 'R' : {
      turn_right();
    }
    break;
    case 'I' : {
      forward_right();
    }
    break;
    case 'J' : {
      backward_right();
    }
    break;
    case 'G' : {
      forward_left();
    }
    break;
    case 'H' : {
      backward_left();
    }
    break;
    default: {
      auto speed_mode = SPEED_MAP_TO_COMMAND[current_command];
      if (speed_mode != -1){
        current_motor_speed = speed_mode * SPEED_QUANT; // TODO: math.ceil()
        Serial.println(speed_mode);
      }
      else{
        //stop();
        nop();
      }
    }
    break;
  }

}

void check_connection(){
  time_without_command = millis() - last_received_signal_timestamp;

  if (time_without_command > 300){
    current_command=' ';
    connection_lost = true;
    stop();

    Serial.print("Disbanding connection: more than 0.3 second without new commands.\n");
  }
}

void receive_command(){
  if (bluetooth_module.available()>0){
    current_command = bluetooth_module.read();

    last_received_signal_timestamp = millis();

    max_loops_without_command=max(max_loops_without_command, loops_without_command*30);
    loops_without_command=0;

    total_commands_received++;
  }
}

void begin_bluetooth_connection(){
  while (!(bluetooth_module.available() > 0)){
    delay(100);
  }
  Serial.print("Connection established.\n");
}

void default_mode(){
  receive_command();
  check_connection();
  execute_command();
}

void loop() {
  if (!connection_lost){
    loops_without_command+=1;
    default_mode();
  }
  else{
    stop();
    Serial.print("Connection has been lost!\n");
    delay(3000);
  }
}