#include <DFRobot_TFmini.h>
#include <Servo.h>

Servo Lidar_Servo;

SoftwareSerial mySerial(2, 3);  // RX, TX

DFRobot_TFmini TFmini;  //외부 Class
uint16_t dist;          //거리값 계산

//속도 제어
int L_MotorSpeed = 220;
int L_S_MotorSpeed = 180;
int R_MotorSpeed = 220;
int R_S_MotorSpeed = 180;

int sensor_val;  //센서 인식값

//자동차 구동
void SmartCar_Go();
void SmartCar_Back();
void SmartCar_Left();
void SmartCar_Right();
void SmartCar_Stop();

//서보모터 구동
int Servo_Move();

//AutoMode, ManualMode
void AutoMode();
void ManualMode();

//모터드라이버 pin
int RightMotor_E_pin = 5;  // 오른쪽 모터의 Enable & PWM
int RightMotor_1_pin = 8;  // 오른쪽 모터 제어선 IN1
int RightMotor_2_pin = 9;  // 오른쪽 모터 제어선 IN2
int LeftMotor_3_pin = 10;  // 왼쪽 모터 제어선 IN3
int LeftMotor_4_pin = 11;  // 왼쪽 모터 제어선 IN4
int LeftMotor_E_pin = 6;   // 왼쪽 모터의 Enable & PWM

void setup() {
  Lidar_Servo.attach(4);

  pinMode(RightMotor_E_pin, OUTPUT);
  pinMode(RightMotor_1_pin, OUTPUT);
  pinMode(RightMotor_2_pin, OUTPUT);
  pinMode(LeftMotor_3_pin, OUTPUT);
  pinMode(LeftMotor_4_pin, OUTPUT);
  pinMode(LeftMotor_E_pin, OUTPUT);

  Serial.begin(9600);
  TFmini.begin(mySerial);
}

void loop() {
  AutoMode();
}

void AutoMode() {
  if (TFmini.measure()) {         //측정이 가능할때
    dist = TFmini.getDistance();  //거리값 받고
    SmartCar_Go();                //default 값은 직진

    Serial.println(dist);

    if (dist <= 15) {
      if (dist <= 7) {
        SmartCar_Back();
        delay(500);
        SmartCar_Stop();
        delay(200);
      } 
      else {
        SmartCar_Stop();
        delay(200);

        sensor_val = Servo_Move();

        if (sensor_val == 0) {
          SmartCar_Stop();
          delay(200);
          SmartCar_Back();
          delay(300);
          SmartCar_Right();
          delay(950);
        } 
        else if (sensor_val == 1) {
          SmartCar_Stop();
          delay(200);
          SmartCar_Back();
          delay(300);
          SmartCar_Left();
          delay(950);
        }
      }
    }
  }
}


void ManualMode() {
  //곧 만들 예정
}

void SmartCar_Stop() {
  digitalWrite(RightMotor_1_pin, LOW);
  digitalWrite(RightMotor_2_pin, LOW);
  digitalWrite(LeftMotor_3_pin, LOW);
  digitalWrite(LeftMotor_4_pin, LOW);
  analogWrite(RightMotor_E_pin, 0);
  analogWrite(LeftMotor_E_pin, 0);
}

void SmartCar_Go() {
  digitalWrite(RightMotor_1_pin, HIGH);
  digitalWrite(RightMotor_2_pin, LOW);
  digitalWrite(LeftMotor_3_pin, HIGH);
  digitalWrite(LeftMotor_4_pin, LOW);
  analogWrite(RightMotor_E_pin, R_MotorSpeed);
  analogWrite(LeftMotor_E_pin, L_MotorSpeed);
}

void SmartCar_Back() {
  digitalWrite(RightMotor_1_pin, LOW);
  digitalWrite(RightMotor_2_pin, HIGH);
  digitalWrite(LeftMotor_3_pin, LOW);
  digitalWrite(LeftMotor_4_pin, HIGH);
  analogWrite(RightMotor_E_pin, R_MotorSpeed);
  analogWrite(LeftMotor_E_pin, L_MotorSpeed);
}

void SmartCar_Left() {
  digitalWrite(RightMotor_1_pin, HIGH);
  digitalWrite(RightMotor_2_pin, LOW);
  digitalWrite(LeftMotor_3_pin, LOW);
  digitalWrite(LeftMotor_4_pin, LOW);
  analogWrite(RightMotor_E_pin, R_S_MotorSpeed);
  analogWrite(LeftMotor_E_pin, L_MotorSpeed);
}

void SmartCar_Right() {
  digitalWrite(RightMotor_1_pin, LOW);
  digitalWrite(RightMotor_2_pin, LOW);
  digitalWrite(LeftMotor_3_pin, HIGH);
  digitalWrite(LeftMotor_4_pin, LOW);
  analogWrite(RightMotor_E_pin, R_MotorSpeed);
  analogWrite(LeftMotor_E_pin, L_S_MotorSpeed);
}

int Servo_Move() {
  int dis_30;
  int dis_150;

  Lidar_Servo.write(20);  //각도 조절
  delay(1300);

  delay(200);
  if (TFmini.measure()) {
    delay(100);
    dis_30 = TFmini.getDistance();
    Serial.print("DIS30 : ");
    Serial.println(dis_30);
  }

  Lidar_Servo.write(160);  //각도 조절
  delay(1300);

  delay(200);
  if (TFmini.measure()) {
    delay(100);
    dis_150 = TFmini.getDistance();
    Serial.print("DIS150 : ");
    Serial.println(dis_150);
  }

  if (dis_30 > dis_150)
    sensor_val = 1;
  else if(dis_30 < dis_150)
    sensor_val = 0;
  
  Lidar_Servo.write(90);  //각도 조절
  delay(1300);

  return sensor_val;
}
