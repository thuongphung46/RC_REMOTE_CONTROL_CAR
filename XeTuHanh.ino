  #include <Arduino_FreeRTOS.h>// ĐÃ SỬA LẦN CUỐI
#include<task.h>
#include<queue.h>
#include <SoftwareSerial.h>
SoftwareSerial HC06(0, 1);
int stop_distance = 12;// Khoảng cách phát hiện vật cản
//Kết nối SRF 05 OR 04
const int trigPin = 11; // kết nối chân trig với chân 11 arduino
const int echoPin = 12; // kết nối chân echo với chân 12 arduino

int stop_distance2 = 10;
const int trigPin2 = A4; // kết nối chân trig với chân 11 arduino
const int echoPin2 = A5;


//L298 kết nối arduino
const int motorA1      = 3;  // kết nối chân IN1 với chân 3 arduino
const int motorA2      = 4;  // kết nối chân IN2 với chân 4 arduino
const int motorAspeed  = 5;  // kết nối chân ENA với chân 5 arduino
const int motorB1      = 7; // kết nối chân IN3 với chân 7 arduino
const int motorB2      = 8; // kết nối chân IN4 với chân 8 arduino
const int motorBspeed  = 6;  // kết nối chân ENB với chân 6 arduino

//kết nối của 3 cảm biến hồng ngoại (dò line )
const int L_S = 9; // cb dò line phải
const int S_S = 2; // cb dò line giữa
const int R_S = 10; //cb dò line trái
int spd = 200; //bien to do
int left_sensor_state;// biến lưu cảm biến hồng ngoại line trái
int s_sensor_state;   // biến lưu cảm biến hồng ngoại line giữa
int right_sensor_state;// biến lưu cảm biến hồng ngoại line phải

char lenh;
char command;
unsigned long duration; // biến đo thời gian
unsigned long duration2; //sua

QueueHandle_t xkhoangcach;
TaskHandle_t xdokhoangcach;
QueueHandle_t xkhoangcach2;
TaskHandle_t xdokhoangcach2;
TaskHandle_t xnhanLenh;
TaskHandle_t xtuHanh;
TaskHandle_t xdieuKhien;

void setup() {
  Serial.begin(115200);
  HC06.begin(9600);
  pinMode(L_S, INPUT); // chân cảm biến khai báo là đầu vào
  pinMode(R_S, INPUT);
  pinMode(S_S, INPUT);
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  pinMode(motorAspeed, OUTPUT);
  pinMode(motorBspeed, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);

  analogWrite(motorAspeed, 90); // tốc độ động cơ a ban đầu 120 ( 0 - 255)
  analogWrite(motorBspeed, 90);// tốc độ động cơ b ban đầu 120 ( 0 - 255)
  
  xkhoangcach = xQueueCreate(1, sizeof(int32_t));
  xkhoangcach2 = xQueueCreate(1, sizeof(int32_t));
  xTaskCreate(nhanLenh, "Nhan Lenh", 64, NULL, 2, &xnhanLenh);
  xTaskCreate(dokhoanhcach, "Sender", 128, NULL, 2, &xdokhoangcach);
  xTaskCreate(dokhoanhcach2, "Sender2", 128, NULL, 2, &xdokhoangcach2); //sua
  xTaskCreate(tuHanh, "Tu hanh", 128, NULL, 1, &xtuHanh);
  xTaskCreate(dieuKhien, "Dieu Khien", 128, NULL, 1, &xdieuKhien);
  vTaskStartScheduler();
  delay(2000);

}
void nhanLenh (void *p) {
  Serial.println("Ham nhan lenh");

  for (; ;) {
    if (Serial.available() > 0) {
      if (HC06.available()) {
        command = (HC06.read());
        Serial.println(command);
        if (command == 'w') {
          vTaskSuspend(xtuHanh);
          vTaskPrioritySet(xdieuKhien, 2);
          vTaskResume(xdieuKhien);
        }
        if (command == 'W') {
          vTaskSuspend(xdieuKhien);
          vTaskPrioritySet(xtuHanh, 2);
          vTaskResume(xtuHanh);
        }

      }
    }
  }
}
void tuHanh(void *p) {
  int32_t value = 0;
  int32_t value2 = 0; //sua
  while (1) {
    xQueuePeek(xkhoangcach, &value, portMAX_DELAY);
     xQueuePeek(xkhoangcach2, &value2, portMAX_DELAY); //sua
    if (value > 18 && value2 < 12) {//không gặp vật cản không gặp hố
      left_sensor_state = digitalRead(L_S);
      s_sensor_state = digitalRead(S_S);
      right_sensor_state = digitalRead(R_S);

      if ((digitalRead(L_S) == 0) && (digitalRead(S_S) == 1) && (digitalRead(R_S) == 0)) {
        forword(); // đi tiến
      }
      if ((digitalRead(L_S) == 1) && (digitalRead(S_S) == 1) && (digitalRead(R_S) == 0)) {
        turnLeft(); // rẻ trái
      }
      if ((digitalRead(L_S) == 1) && (digitalRead(S_S) == 0) && (digitalRead(R_S) == 0)) {
        turnLeft(); // rẻ trái
      }
      if ((digitalRead(L_S) == 0) && (digitalRead(S_S) == 1) && (digitalRead(R_S) == 1)) {
        turnRight(); // rẻ phải
      }
      if ((digitalRead(L_S) == 0) && (digitalRead(S_S) == 0) && (digitalRead(R_S) == 1)) {
        turnRight(); // rẻ phải
      }
      if ((digitalRead(L_S) == 1) && (digitalRead(S_S) == 1) && (digitalRead(R_S) == 1)) {
        Stop(); // stop
      }
    }else if(value2 > 12 && value > 18){
      tranhho();
    }else if(value2 > 12 && value < 18){//gặp hố và gặp vật cản
      tranhho();
    }else if(value2 <12 && value < 18){ //ko hố và có vật cản
      tranhvatcan();
    }
    else{
      tranhho();
    }
      
  }
}

void dieuKhien(void *p) {
   int32_t value = 0;
  for (;;) {
    
     xQueuePeek(xkhoangcach, &value, portMAX_DELAY);
    Serial.println("Ham dieu khien");
    if(command == '0'){
      spd = 70;      
    }
     if(command == '1'){
      spd = 80;      
    }
     if(command == '2'){
      spd = 100;      
    }
     if(command == '3'){
      spd = 110;      
    }
     if(command == '4'){
      spd = 127;      
    }
     if(command == '5'){
      spd = 140;      
    }
     if(command == '6'){
      spd = 160;      
    }
     if(command == '7'){
      spd = 200;      
    }
     if(command == '8'){
      spd = 230;      
    }
     if(command == '9'){
      spd = 255;      
    }
    
    if (command == 'F') {
      if(value>25){
      forword();
      }else{Stop();}
    }
    if (command == 'B') {
      back();
    }

    if (command == 'L') {
      turnLeft();
    }
    if (command == 'R') {
      turnRight();
    }
    if (command == 'S') {
      Stop();
    }
  }
}
//A1,B1 ben trai
void tranhho(){ //sua
  digitalWrite (motorA1, LOW);
  digitalWrite(motorA2, LOW);
  digitalWrite (motorB1, LOW);
  digitalWrite(motorB2, LOW);
  delay(100);
  
  digitalWrite (motorA1, HIGH); // cho xe robot chạy lùi 1 đoạn
  digitalWrite(motorA2, LOW);
  digitalWrite (motorB1, LOW);
  digitalWrite(motorB2, HIGH);
  delay(350);

  digitalWrite (motorA1, LOW);
  digitalWrite(motorA2, LOW);
  digitalWrite (motorB1, LOW);
  digitalWrite(motorB2, LOW);
  delay(100);
  
  digitalWrite (motorA1, HIGH); // cho xe robot xoay sang trái
  digitalWrite(motorA2, LOW);
  digitalWrite (motorB1, HIGH);
  digitalWrite(motorB2, LOW);
  delay(1100);

    digitalWrite (motorA1, LOW); //cho xe robot đi thẳng 1
    digitalWrite(motorA2, HIGH);
    digitalWrite (motorB1, HIGH);
    digitalWrite(motorB2, LOW);


      
  while (left_sensor_state == LOW){ 
      left_sensor_state = digitalRead(L_S);
      s_sensor_state = digitalRead(S_S);
      right_sensor_state = digitalRead(R_S);
      Serial.println("in the first while");
    }  
} //sua

void tranhvatcan(){
    
    digitalWrite (motorA1, HIGH); // cho xe robot xoay phải
    digitalWrite(motorA2, LOW);
    digitalWrite (motorB1, HIGH);
    digitalWrite(motorB2, LOW);
    delay(400);
    digitalWrite (motorA1, LOW);
    digitalWrite(motorA2, LOW);
    digitalWrite (motorB1, LOW);
    digitalWrite(motorB2, LOW);
    delay(200);

    digitalWrite (motorA1, LOW); //cho xe robot đi thẳng 1 đoạn
    digitalWrite(motorA2, HIGH);
    digitalWrite (motorB1, HIGH);
    digitalWrite(motorB2, LOW);
    delay(600);
    digitalWrite (motorA1, LOW);
    digitalWrite(motorA2, LOW);
    digitalWrite (motorB1, LOW);
    digitalWrite(motorB2, LOW);
    delay(200);


    digitalWrite (motorA1, LOW); // cho xe robot xoay trái 1 đoạn
    digitalWrite(motorA2, HIGH);
    digitalWrite (motorB1, LOW);
    digitalWrite(motorB2, LOW);
    delay(550);
    digitalWrite (motorA1, LOW);
    digitalWrite(motorA2, LOW);
    digitalWrite (motorB1, LOW);
    digitalWrite(motorB2, LOW);
    delay(200);

    ///////////////////
    digitalWrite (motorA1, LOW); //cho xe robot đi thẳng 1 đoạn
    digitalWrite(motorA2, HIGH);
    digitalWrite (motorB1, HIGH);
    digitalWrite(motorB2, LOW);
    delay(500);
    digitalWrite (motorA1, LOW);
    digitalWrite(motorA2, LOW);
    digitalWrite (motorB1, LOW);
    digitalWrite(motorB2, LOW);
    delay(200); 

    ////////////////////////
    digitalWrite (motorA1, LOW); //cho xe robot xoay trái 1 đoạn
    digitalWrite(motorA2, HIGH);
    digitalWrite (motorB1, LOW);
    digitalWrite(motorB2, HIGH);
    delay(600);

    digitalWrite (motorA1, LOW);
    digitalWrite(motorA2, LOW);
    digitalWrite (motorB1, LOW);
    digitalWrite(motorB2, LOW);
    delay(200);

    digitalWrite (motorA1, LOW); //cho xe robot đi thẳng 1
    digitalWrite(motorA2, HIGH);
    digitalWrite (motorB1, HIGH);
    digitalWrite(motorB2, LOW);


    while (left_sensor_state == LOW) {

      left_sensor_state = digitalRead(L_S);
      s_sensor_state = digitalRead(S_S);
      right_sensor_state = digitalRead(R_S);
      Serial.println("in the first while");
    }  
}
void dokhoanhcach(void*p)
{
  int32_t  distance;
  while (1) {
    digitalWrite(trigPin, 0);  // tắt chân trig
    delayMicroseconds(2);
    digitalWrite(trigPin, 1);  // phát xung từ chân trig
    delayMicroseconds(5);   // xung có độ dài 5 microSeconds
    digitalWrite(trigPin, 0);  // tắt chân trig
    duration = pulseIn(echoPin, HIGH);
    distance = int(duration / 2 / 29.412);
    Serial.println(distance);
    xQueueOverwrite(xkhoangcach, &distance);
    vTaskDelay(0.1);
    
  }
}

void dokhoanhcach2(void*p) //sua
{
  int32_t  distance2;
  while (1) {
    digitalWrite(trigPin2, 0);  // tắt chân trig
    delayMicroseconds(2);
    digitalWrite(trigPin2, 1);  // phát xung từ chân trig
    delayMicroseconds(5);   // xung có độ dài 5 microSeconds
    digitalWrite(trigPin2, 0);  // tắt chân trig
    duration2 = pulseIn(echoPin2, HIGH);
    distance2 = int(duration2 / 2 / 29.412);
    Serial.println(distance2);
    xQueueOverwrite(xkhoangcach2, &distance2);
    vTaskDelay(0.1);
    
  }
}



void loop() {
}

void forword() { // chương trình con xe robot đi tiến

  digitalWrite (motorA1, LOW);
  digitalWrite(motorA2, HIGH);
  digitalWrite (motorB1, HIGH);
  digitalWrite(motorB2, LOW);
}
void back() {
  digitalWrite (motorA1, HIGH); // cho xe robot chạy lùi 1 đoạn
  digitalWrite(motorA2, LOW);
  digitalWrite (motorB1, LOW);
  digitalWrite(motorB2, HIGH);
}

void turnRight() {

  digitalWrite (motorA1, LOW);
  digitalWrite(motorA2, LOW);
  digitalWrite (motorB1, HIGH);
  digitalWrite(motorB2, LOW);
}

void turnLeft() {

  digitalWrite (motorA1, LOW);
  digitalWrite(motorA2, HIGH);
  digitalWrite (motorB1, LOW);
  digitalWrite(motorB2, LOW);
}

void Stop() {

  digitalWrite (motorA1, LOW);
  digitalWrite(motorA2, LOW);
  digitalWrite (motorB1, LOW);
  digitalWrite(motorB2, LOW);
}
