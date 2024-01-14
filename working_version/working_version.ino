#include <SoftwareSerial.h>

SoftwareSerial bluetooth_module(51, 49); //51 49 RX | TX pins for bluetooth module.

const int BLUETOOTH_BAUDRATE = 9600;
const int MASTER_DEVICE_BAUDRATE = 9600;

/*
Initialising constants responsible for engine speed and direction control pins 
*/
const int ENGINE1_SPEED_READ_PIN = 18;
const int ENGINE2_SPEED_READ_PIN = 19;
const int ENGINE3_SPEED_READ_PIN = 20;
const int ENGINE4_SPEED_READ_PIN = 21;

const int ENGINE1_BRAKE_PIN = 23; // Overlaps with e4speedpin
const int ENGINE2_BRAKE_PIN = 25;
const int ENGINE3_BRAKE_PIN = 22;
const int ENGINE4_BRAKE_PIN = 24;

const int ENGINE1_SPEED_CONTROL_PIN=4;
const int ENGINE2_SPEED_CONTROL_PIN=5;
const int ENGINE3_SPEED_CONTROL_PIN=6;
const int ENGINE4_SPEED_CONTROL_PIN=7;

// const int ENGINE1_SPEED_READ_PIN = 18;
// const int ENGINE2_SPEED_READ_PIN = 19;
// const int ENGINE3_SPEED_READ_PIN = 20;
// const int ENGINE4_SPEED_READ_PIN = 21;

volatile long long engine1_last_pulse = 1;
volatile long  engine2_last_pulse = 1;
volatile long long engine3_last_pulse = 1;
volatile long long engine4_last_pulse = 1;

volatile double engine1_speed = 0;
volatile double engine2_speed = 0;
volatile double engine3_speed = 0;
volatile double engine4_speed = 0;

volatile long  current_time = 0;

const int ENGINE1_DIRECTION_CONTROL_PIN=29;
const int ENGINE2_DIRECTION_CONTROL_PIN=31;
const int ENGINE3_DIRECTION_CONTROL_PIN=30;
const int ENGINE4_DIRECTION_CONTROL_PIN=32;

constexpr double MAX_SPEED = 255;
const int MOTOR_SPEED_0 = 0;
const int TEST_SPEED = 120; // Out of 255 (?)
const int MOTOR_SPEED_1 = 130;
const int MOTOR_SPEED_2 = 150;
const int MOTOR_SPEED_3 = 170;

constexpr int MAX_SPEED_MODE = 10;
constexpr double SPEED_QUANT = MAX_SPEED / MAX_SPEED_MODE;

static int SPEED_MAP_TO_COMMAND[256]; 

constexpr size_t CHAR_MIN_SIZE = 0;
constexpr size_t CHAR_MAX_SIZE = 255;


const int TURNING_DIRECTION_PIN_1 = 13;
const int TURNING_DIRECTION_PIN_2 = 11;

const int TURNING_SPEED_PIN_1 = 12;
const int TURNING_SPEED_PIN_2 = 10;

const int V_5=11;
const int V_5_1=53;

char current_command = ' '; // Contains a command symbol received with bluetooth.

int current_motor_speed = TEST_SPEED;

constexpr unsigned int SPEED_BASE = 10000;

double TEMP1_DIF;

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
  
  pinMode(V_5, OUTPUT);
  pinMode(V_5_1, OUTPUT);
  digitalWrite(V_5, HIGH);
  digitalWrite(V_5_1, HIGH);

  pinMode(ENGINE1_DIRECTION_CONTROL_PIN, OUTPUT);
  pinMode(ENGINE2_DIRECTION_CONTROL_PIN, OUTPUT);
  pinMode(ENGINE3_DIRECTION_CONTROL_PIN, OUTPUT);
  pinMode(ENGINE4_DIRECTION_CONTROL_PIN, OUTPUT);

  // attachInterrupt(digitalPinToInterrupt(ENGINE1_SPEED_READ_PIN), engine1_speed_signal, RISING);
  // attachInterrupt(digitalPinToInterrupt(ENGINE2_SPEED_READ_PIN), engine2_speed_signal, RISING);
  // attachInterrupt(digitalPinToInterrupt(ENGINE3_SPEED_READ_PIN), engine3_speed_signal, RISING);
  // attachInterrupt(digitalPinToInterrupt(ENGINE4_SPEED_READ_PIN), engine4_speed_signal, RISING);

  no_turn(); // Don't delete/comment, as wheels start turning immediately on one side. (probably)
  Serial.print("Init!!\n");

  Serial.begin(MASTER_DEVICE_BAUDRATE);
  bluetooth_module.begin(BLUETOOTH_BAUDRATE);

  begin_bluetooth_connection(); // Waits until bluetooth connection is established.
}


void loop()
{

  // turn_left();
  // delay(1000);
  
  // delay(1000);
  // move_forward();
  // delay(1000);
  // current_motor_speed = 50;
  // move_forward();
  // delay(5000);
  // delay(1000);
  // block_wheels();
  // delay(1000);
  // for (size_t speed = 0; speed < 255; speed++){
  //   current_motor_speed = speed;
  //   move_forward();
  //   delay(50);
  // }
  
  // unblock_wheels();
  // current_motor_speed = 255;
  // move_forward();
  // delay(5000);

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

void default_mode(){
  receive_command();
  check_connection();
  execute_command();
  // Serial.print("Engine 1 speed: ");
  // Serial.print(engine1_speed);
  // Serial.print("\n");
  // Serial.print("Engine 2 speed: ");
  // Serial.print(engine2_speed);
  // Serial.print("\n");
}


void begin_bluetooth_connection(){
  while (!(bluetooth_module.available() > 0)){
    delay(100);
  }
  Serial.print("Connection established.\n");
}


void receive_command(){
  if (bluetooth_module.available()>0){
    current_command = bluetooth_module.read();

    last_received_signal_timestamp = millis();

    max_loops_without_command=max(max_loops_without_command, loops_without_command*30);
    loops_without_command=0;

    total_commands_received++;


    /*Serial.print(current_command);
    Serial.print("\nLoops before command: ");
    Serial.print(loops_without_command);
    Serial.print("\n");*/
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


void execute_command(){
  // if (current_command != 'S'){
  //   Serial.print(current_command);
  // }
  
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


void no_turn(){
  digitalWrite(TURNING_DIRECTION_PIN_1, LOW);
  digitalWrite(TURNING_DIRECTION_PIN_2, LOW);

  digitalWrite(TURNING_SPEED_PIN_1, LOW);
  digitalWrite(TURNING_SPEED_PIN_2, LOW);
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

void engine1_speed_signal(){
  current_time = micros();
  engine1_speed = 1 / (current_time - engine1_last_pulse);
  engine1_last_pulse = current_time;  
}

void engine2_speed_signal(){
  current_time = micros();
  TEMP1_DIF = current_time - engine2_last_pulse;
  engine2_speed = SPEED_BASE / (double) TEMP1_DIF;
  Serial.println(engine2_speed);
  
  // Serial.println(engine2_speed);
  
  // long long temp = current_time - engine3_last_pulse;
  // Serial.println("Pulse from engine 2!");

  // Serial.print (int(engine2_speed));  //prints the int part
  // Serial.print("."); // print the decimal point
  // unsigned int frac;
  // long precision = 1000;
  // if(engine2_speed >= 0){
  //   frac = (engine2_speed - int(engine2_speed)) * precision;
  // }
  // else{
  //   frac = (int(engine2_speed)- engine2_speed ) * precision;
  // }
  // Serial.println(frac,DEC) ;

  // Serial.println(engine2_speed);
  engine2_last_pulse = current_time;
}

void engine3_speed_signal(){
  current_time = micros();
  engine3_speed = 1 / (current_time - engine3_last_pulse);
  engine3_last_pulse = current_time;
}

void engine4_speed_signal(){
  current_time = micros();
  engine4_speed = 1 / (current_time - engine4_last_pulse);
  engine4_last_pulse = current_time;
}