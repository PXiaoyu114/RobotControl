#include <Servo.h>

// 舵机数据
Servo servo[4];
//0:base, 1:fArm, 2:rArm, 3:claw
const int servoPin[4] = {9,8,7,6};
const int servoMin[4] = {0,30,25,30};
const int servoMax[4] = {180,100,160,175};
int servoAngle[4] = {90,90,90,90};

// 定义左摇杆引脚
const int LEFT_X = A0;
const int LEFT_Y = A1;
const int LEFT_SW = 0; // 数字引脚 D0

// 定义右摇杆引脚
const int RIGHT_X = A2;
const int RIGHT_Y = A3;
const int RIGHT_SW = 1; // 数字引脚 D1

// 延迟时间
int delaytime = 7;
// 摇杆识别的起始偏移量
int joystickStart = 300;

// 函数：摇杆超过一定角度时，返回判定值
int joystickAct(int joystick){
  if(joystick < 512 - joystickStart)
    return -1;
  else if(joystick > 512 + joystickStart)
    return 1;
  else
    return 0;
}

// 函数:当摇杆超过判定值时，增加对应舵机角度
void servoChange(int num,int dir){
  if(dir > 0 && servoAngle[num] < servoMax[num]){
    servoAngle[num]++;
  }
  else if(dir < 0 && servoAngle[num] > servoMin[num]){
    servoAngle[num]--;
  }
}

// 函数：读取舵机角度并显示在串口监视器
void reportAngles() {
  Serial.print("Base: ");
  Serial.print(servoAngle[0]);
  Serial.print(" | fArm: ");
  Serial.print(servoAngle[1]);
  Serial.print(" | rArm: ");
  Serial.print(servoAngle[2]);
  Serial.print(" | claw: ");
  Serial.println(servoAngle[3]);
}

void setup() {

  //设定舵机引脚
  for(int i=0;i<4;i++){
    servo[i].attach(servoPin[i]);
    delay(200);
  }

  //设定初始角度
  for(int i=0;i<4;i++){
    servo[i].write(servoAngle[i]);
    delay(10);
  }

  Serial.begin(9600);
  Serial.println("Start");
}

void loop() {

  // 读取模拟量（X/Y轴）
  int lx = analogRead(LEFT_X);
  int ly = analogRead(LEFT_Y);
  int rx = analogRead(RIGHT_X);
  int ry = analogRead(RIGHT_Y);
  
  // 读取数字量（按键按下状态）
  int lSw = digitalRead(LEFT_SW);
  int rSw = digitalRead(RIGHT_SW);

  // 摇杆输入到函数
  servoChange(0,joystickAct(lx));
  servoChange(1,joystickAct(ly));
  servoChange(2,-joystickAct(ry));
  servoChange(3,joystickAct(rx));

  // 舵机角度执行
  for(int i=0;i<4;i++){
    servo[i].write(servoAngle[i]);
  }
  delay(delaytime);

  // 串口通信
  // static unsigned long lastPrintTime = 0; 
  // if (millis() - lastPrintTime >= 500) { // 每500毫秒执行一次
  //   reportAngles();
  //   lastPrintTime = millis(); // 更新时间戳
  // }
}
