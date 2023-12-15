#include <Servo.h>
#include <SoftwareSerial.h>

Servo Lidar_Servo;

int e_front = 2;
int t_front = 3;
int dist = 0;

SoftwareSerial BLUETOOTH(12, 13);
int mode_change = 0;  //ManualMode, AutoMode 변경시 이용

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

//초음파 센서
int Ultrasonic();

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
  pinMode(e_front, INPUT);
  pinMode(t_front, OUTPUT);

  Serial.begin(9600);
  BLUETOOTH.begin(9600);
  Serial.println("CONNECT");
}

void loop() {
  if (!mode_change) {
    switch (BLUETOOTH.read()) {
      case 'm': //ManualMode
        ManualMode();
        mode_change = false;
        break;
      case 'a': //AutoMode
        SmartCar_Stop();
        mode_change = true;
        break;
    }
  } else {
    if ('m' == BLUETOOTH.read()) {
      mode_change = false;
      SmartCar_Stop();
      return;
    }
    else if('t' == BLUETOOTH.read()){
      SmartCar_Stop();  //특정 event 발생시 자동차 일시 정지
      delay(2000);  //2초간
      //이후 else 문 추가해서 특정 event 더 발생하도록
    }
    else {
      AutoMode();
    }
  }
}

void AutoMode() {
  dist = Ultrasonic();  //거리값 받고
  SmartCar_Go();
  Serial.println(dist);

  if (dist <= 200) {
    if (dist <= 50) {
      SmartCar_Back();
      delay(500);
      SmartCar_Stop();
      delay(200);
    } else {
      SmartCar_Stop();
      delay(200);

      sensor_val = Servo_Move();
      if (sensor_val == 0) {
        SmartCar_Stop();
        delay(200);
        SmartCar_Back();
        delay(450);
        SmartCar_Left();
        delay(950);
      } else if (sensor_val == 1) {
        SmartCar_Stop();
        delay(200);
        SmartCar_Back();
        delay(450);
        SmartCar_Right();
        delay(950);
      }
    }
  }
}


void ManualMode() {
  if (mode_change == 'f')
    SmartCar_Go();
  else if (mode_change == 'l')
    SmartCar_Left();
  else if (mode_change == 'r')
    SmartCar_Right();
  else if (mode_change == 'b')
    SmartCar_Back();
  else if (mode_change == 's')
    SmartCar_Stop();
}
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
  analogWrite(LeftMotor_E_pin, 100);
}

void SmartCar_Right() {
  digitalWrite(RightMotor_1_pin, LOW);
  digitalWrite(RightMotor_2_pin, LOW);
  digitalWrite(LeftMotor_3_pin, HIGH);
  digitalWrite(LeftMotor_4_pin, LOW);
  analogWrite(RightMotor_E_pin, 100);
  analogWrite(LeftMotor_E_pin, L_S_MotorSpeed);
}

int Ultrasonic() {
  long duration, distance;
  digitalWrite(t_front, HIGH);  // trigPin에서 초음파 발생(echoPin도 HIGH)
  delayMicroseconds(10);
  digitalWrite(t_front, LOW);
  duration = pulseIn(e_front, HIGH);  // echoPin 이 HIGH를 유지한 시간을 저장 한다.
  distance = ((float)(340 * duration) / 1000) / 2;

  return distance;
}

int Servo_Move() {
  Lidar_Servo.write(30);  //각도 조절
  delay(500);

  int dis_30 = Ultrasonic();
  delay(500);

  Lidar_Servo.write(150);  //각도 조절
  delay(500);

  int dis_150 = Ultrasonic();
  delay(500);

  if (dis_30 > dis_150)
    sensor_val = 1;
  else
    sensor_val = 0;

  Lidar_Servo.write(90);  //각도 조절
  delay(500);

  return sensor_val;
}
