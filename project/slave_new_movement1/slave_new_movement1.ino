#include "GY_85.h"
#include <Wire.h>
#include <SPI.h>
#include "RF24.h"

#define button 2
#define move_length 8
#define TO_DEG(x) (x * 57.2957795131)  // *180/pi
const int ledPin = 4;
const int soundPin = 3;
const int margin = 20;
const int marginSum = 100; //同位置的x, y, z總共所能容忍的誤差量
const int marginEach = 35; //每個x, y, z個別容忍的誤差量
const uint64_t rAddress[] = {0x7878787878LL, 0xB3B4B5B6F1LL, 0xB3B4B5B6CDLL, 0xB3B4B5B6A3LL, 0xB3B4B5B60FLL, 0xB3B4B5B605LL };
const byte pipe = 1;  // 指定通道編號

RF24 rf24(7, 8); // CE腳, CSN腳
GY_85 GY85;     //create the object

bool record = false; // If button has been pressed and the movement has been start recording.
bool record_finish = false;
bool rise[3] = {false}; // If the phase of an axis is rising (from -90 to 90 degree).
bool firstInsert[3] = {true};
int movement[3][move_length] = {0}; // Three array for different axis.
int movementC[3][move_length] = {0};
int move_num[3] = {0};
int temp[3] = {0};
int phase[3];
int x[8] = {0};
int y[8] = {0};
int z[8] = {0};
byte movementTorF; //0 no result, 1 false, 2 true.

void setup() {
  
  Serial.begin(9600);
  rf24.begin();
  rf24.setChannel(82);  // 設定頻道編號
  rf24.setPALevel(RF24_PA_MAX);
  rf24.setDataRate(RF24_2MBPS);
  rf24.openReadingPipe(pipe, rAddress[0]);  // 開啟通道和位址
  rf24.startListening();  // 開始監聽無線廣播
  Serial.println("nRF24L01 ready!");
  Wire.begin();
  GY85.init();

  pinMode(button, INPUT_PULLUP); //設定button為高電位，若按下button則拉低電位
  pinMode(ledPin, OUTPUT);
  pinMode(soundPin, OUTPUT);

  // Reset movement array to -99 degree.
  int i, j;
  for (i = 0; i < 3; i++) {
    for (j = 0; j < move_length; j++) {
      movement[i][j] = -99;
      movementC[i][j] = -99;
    }
  }
}

void loop() {

  delay(100); // Sampling every 0.1s

  // #0
  if (rf24.available(&pipe)) {
    int i = 0;
    while (i < 3) {
      int sizeC = sizeof(movementC[i]);
      rf24.read(&movementC[i], sizeC);
      i++;
    }
    i = 0;
    Serial.println("Receive:");
    for (int i = 0; i < 3; i++) {
      for(int y = 0; y < 8; y++) { 
        Serial.print(", ");
        Serial.print(movementC[i][y]);
      }
        Serial.println();
    }
    Serial.println();
  }
  // #1
  bool button_press = digitalRead(button); //讀button的狀態
  
  if (button_press == LOW) {
    Serial.println("button press");
    if (record) { // If it's recording, turn it off.
      Serial.println("finish");
      temp[0] = 0;
      temp[1] = 0;
      temp[2] = 0;
      record = false;
      record_finish = true;
    } else { // If it's not recording, turn it on.
      Serial.println("start");
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
    while (k != move_length) {
      Serial.print(movement[0][k]);
      if (k != move_length - 1) Serial.print(", ");
      k++;
    }
    Serial.println(" }");
    
    // Y:
    Serial.print(" y:{ ");
    k = 0;
    while(k != move_length) {
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
    Serial.println( " }" );
    Serial.println();
    
    Serial.println("Receive:");
    for (int i = 0; i < 3; i++) {
      for(int y = 0; y < 8; y++)
      { 
        Serial.print(", ");
        Serial.print(movementC[i][y]);
      }
        Serial.println();
    }
    compare(); // Compare two array ( movement[], movementC[] ).
    
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
    float ax2, ay2, az2;
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
  else if (rise[axis] == true) {
    if (phase[axis] - temp[axis] >= 0) {
      temp[axis] = phase[axis];
    } else if (temp[axis] - phase[axis] > margin) {
      movement[axis][move_num[axis]] = temp[axis];
      rise[axis] = false;
      temp[axis] = phase[axis];
      move_num[axis]++;
    } else {}
  } else {
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

void compare () {
  Serial.print("Enter compare");
  int sum, sumC, eachError[3]; 
  for (int i = 0; i < move_length; i++) {
    Serial.println("Enter for loop");
    for (int j = 0; j < 3; j++) { //同一位置的xyz總和
      sum += abs(movement[j][i]);
      sumC += abs(movementC[j][i]);
      eachError[j] = abs(movement[j][i] - movementC[j][i]);
      Serial.print("eachError:");
      Serial.println(eachError[j]);
    }
    if (abs(sum-sumC) <= marginSum
      && eachError[0] <= marginEach
      && eachError[1] <= marginEach
      && eachError[2] <= marginEach) {
      movementTorF = 2; //true  
    } else {
      movementTorF = 1;
    }
    if (movementTorF == 1) break; //有任何一次不合格，就跳出迴圈，判斷為動作錯誤
    sum = 0;
    sumC = 0;
  }
  if (movementTorF == 1) movementFalse();
  else if (movementTorF == 2) movementTrue();
  else Serial.println("Something wrong!"); //理論上不會出現的狀況
}

void movementTrue () {
  Serial.println("Movement True~");
  for (int i = 0; i < 10; i++) {
    digitalWrite(ledPin, HIGH);
    delay(50);
    digitalWrite(ledPin, LOW);
    delay(50);
  }
}

void movementFalse () {
    Serial.println("Movement False!!!");
    for (int i = 0; i < 1; i++) {
    tone(soundPin, 523, 50);
    delay(100);
    tone(soundPin, 1046, 50);
    delay(100);
  }
}
