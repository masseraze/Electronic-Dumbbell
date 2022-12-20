#include "GY_85.h"
#include <Wire.h>
#include <SPI.h>
#include "RF24.h"

#define button 2
#define move_length 8
const int soundPin = A0;
const int margin = 20;
const uint64_t rAddress[] = {0x7878787878LL, 0xB3B4B5B6F1LL, 0xB3B4B5B6CDLL, 0xB3B4B5B6A3LL, 0xB3B4B5B60FLL, 0xB3B4B5B605LL };

RF24 rf24(7, 8); // CE腳, CSN腳
GY_85 GY85;     //create the object

bool record = false; // If button has been pressed and the movement has been start recording.
bool record_finish = false;
bool rise[3] = {false}; // If the phase of an axis is rising (from -90 to 90 degree).
bool firstInsert[3] = {true};
int movement[3][move_length] = {0}; // Three array for different axis.
int move_num[3] = {0};
int temp[3] = {0};
int phase[3];
int x[8] = {0};
int y[8] = {0};
int z[8] = {0};

// below are for transmission

void setup() {
  pinMode(button, INPUT_PULLUP); //設定button為高電位，若按下button則拉低電位   
  pinMode(soundPin, OUTPUT);
  rf24.begin();
  rf24.setChannel(83);       // 設定頻道編號
  rf24.openWritingPipe(rAddress[0]); // 設定通道位址
  rf24.setPALevel(RF24_PA_MAX);
  rf24.setDataRate(RF24_2MBPS);
  rf24.stopListening();       // 停止偵聽；設定成發射模式
  Wire.begin();
  Serial.begin(9600);
  GY85.init();

  // Reset movement array to -99 degree.
  int i, j;
  for (i = 0; i < 3; i++) {
    for (j = 0; j < move_length; j++) {
      movement[i][j] = -99;
    }
  }   
}

void loop() {    
  delay(100); // Sampling every 0.1s
   
  // #1
  bool button_press = digitalRead(button); //讀button的狀態
  
  if (button_press == LOW) { 
    if (record) { // If it's recording, turn it off.
      Serial.println("finish");
      temp[0] = 0;
      temp[1] = 0;
      temp[2] = 0;
      record = false;
      record_finish = true;
      tone(soundPin, 300, 50);
    }
    else { // If it's not recording, turn it on.
      Serial.println("start");
      tone(soundPin, 1200, 50);
      delay(100);
      tone(soundPin, 888, 50);
      delay(100);
      record = true;
      record_finish = false;
      get_phase();
      movement[0][0] = phase[0]; // X-axis
      movement[1][0] = phase[1]; // Y-axis
      movement[2][0] = phase[2]; // Z-axis
      temp[0] = phase[0];
      temp[1] = phase[1];
      temp[2] = phase[2];        
      move_num [0] = 1;
      move_num [1] = 1;
      move_num [2] = 1;
      firstInsert[0] = true;
      firstInsert[1] = true;
      firstInsert[2] = true;
    }
    delay(100); 
  }

  // #2
  if (record) {
    get_phase();
    insert(0);
    insert(1);
    insert(2);
  }

  // #3
  if (record_finish) {
      
    int k;
    // X
    Serial.print(" x:{ ");
    k = 0;
    while(k != move_length) {
      Serial.print(movement[0][k]);
      if (k != move_length - 1) Serial.print(", ");
      k++;
    }
    Serial.println(" }");
    
    // Y:
    Serial.print(" y:{ ");
    k = 0;
    while (k != move_length) {
      Serial.print(movement[1][k]);
      if (k != move_length - 1) Serial.print(", ");
      k++;
    }
    Serial.println(" }");
    
    // Z:
    Serial.print(" z:{ ");
    k = 0;
    while(k != move_length) {
      Serial.print(movement[2][k]);
      if (k != move_length - 1) Serial.print(", ");
      k++;
    }
    Serial.println(" }");
    Serial.println();
    
    for(int u = 0; u < 8; u++) {
      x[u] = movement[0][u];
      y[u] = movement[1][u];
      z[u] = movement[2][u];
    }
    rf24.write(&x, 16); // X-axis
    rf24.write(&y, 16); // Y-axis
    rf24.write(&z, 16); // Z-axis

    // Reset movement array to -99 degree.
    int i, j;
    for (i = 0; i < 3; i++) {
      for (j = 0; j < move_length; j++) {
       movement[i][j] = -99;
      }
    }
    record_finish = false;
  }       
}

void get_phase() {
  
  int ax = GY85.accelerometer_x(GY85.readFromAccelerometer());
  int ay = GY85.accelerometer_y(GY85.readFromAccelerometer());
  int az = GY85.accelerometer_z(GY85.readFromAccelerometer());

  // 從靈敏度回推加速度
  float ax2,ay2,az2;
  ax2 = float(ax) / 16384;
  ay2 = float(ay) / 16384;
  az2 = float(az) / 16384;

  // 換算三軸角度
  phase[0] = atan(ax2 / sqrt((ay2 * ay2) + (az2 * az2))) * (180 / M_PI);
  phase[1] = atan(ay2 / sqrt((ax2 * ax2) + (az2 * az2))) * (180 / M_PI);
  phase[2] = atan(az2 / sqrt((ax2 * ax2) + (ay2 * ay2))) * (180 / M_PI);
}

void insert (int axis) {
  if (firstInsert[axis] == true) {
    if (abs(phase[axis] - movement[axis][0]) > margin) {
      if (phase[axis] - movement[axis][0] >= 0) {
        rise[axis] = true;
      } else {
        rise[axis] = false;
      }
      temp[axis] = phase[axis];
      firstInsert[axis] = false;
    }
  }
  if (rise[axis] == true && firstInsert[axis] == false) {
    if (phase[axis] - temp[axis] >= 0) {
      temp[axis] = phase[axis];
    } else if (temp[axis] - phase[axis] > margin) {
      movement[axis][move_num[axis]] = temp[axis];
      rise[axis] = false;
      temp[axis] = phase[axis];
      move_num[axis]++;
    } else {}
  } else if (rise[axis] == false && firstInsert[axis] == false) {
    if (temp[axis] - phase[axis] >= 0) {
      temp[axis] = phase[axis];
    } else if ( phase[axis] - temp[axis] > margin) {
      movement[axis][move_num[axis]] = temp[axis];
      rise[axis] = true;
      temp[axis] = phase[axis];
      move_num[axis]++;
    } else {}
  }
}
