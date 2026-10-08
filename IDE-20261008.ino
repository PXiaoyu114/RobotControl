#include <Servo.h>

// 按钮相关
int lastlSw = 1;
int lastrSw = 1;

// 任务二相关
char taskState = 'P'; // 任务完成状态变量
int taskStep = 0; // 任务步数
unsigned long taskTime; // 任务计时器

// 任务三相关
int times1 = 0; // 任务三（按键1）计数器
typedef struct RecordData{
  byte angles[4];
} RecordData;
const int maxPoint = 200;
RecordData recordData[maxPoint];
int pointCount = 0;
int playCount = 0;
unsigned long lastRecordTime = 0;
bool isRecord = false;
bool isPlay = false;

// 任务五相关
bool isTrack = false;
int penHeight = 70; // 握笔时钩爪高度
int trackStep = 0; // 在一段位移中已进行的步数
int trackPhase = 0; // 0：准备阶段，移动到目标点；1：移动完成
int trackNumT = 0; // 总共点位数
int trackNum = 0; // 当前点位数
unsigned long trackPhaseTime = 0; // 准备阶段等待毫秒数
typedef struct TrackData{
  int pos[3];
} TrackData;
const int maxPos = 5;
TrackData trackData[maxPos];

// 定义R2D与D2R用于弧度与角度转换
const float R2D = 180.0f / PI;
const float D2R = PI / 180.0f;

// 舵机数据
Servo servo[4];
//0:base, 1:fArm, 2:rArm, 3:claw
const int servoPin[4] = {9,8,7,6};
const int servoMin[4] = {0,0,25,30};
const int servoMax[4] = {180,120,180,175};
int servoAngle[4] = {90,90,90,90};
int servoAngleT[4] = {90,90,90,90};

//机械臂夹爪坐标
float x = 0;
float y = 0;
float z = 0;

// 机械臂尺寸
const float L1 = 80;
const float L2 = 80;
const float h = 10;
const float l = 50;
const float L3 = 14;
const float H = 60;

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
int joystickStart = 400;

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
  if(dir > 0 && servoAngleT[num] < servoMax[num]){
    servoAngleT[num]++;
  }
  else if(dir < 0 && servoAngleT[num] > servoMin[num]){
    servoAngleT[num]--;
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
  Serial.print(servoAngle[3]);
  incoordinate(servoAngle[0],servoAngle[1],servoAngle[2]);
  Serial.print(" | x: ");
  Serial.print(x);
  Serial.print(" y: ");
  Serial.print(y);
  Serial.print(" z: ");
  Serial.println(z);
}

// 函数：坐标反解为舵机角度
int coordinate(float x,float y,float z){
  float a3 = atan2f(y,x);
  float R = sqrtf(x*x + y*y);
  float L = R - L3 - l;
  float Z = z - H + h;
  float D = sqrtf(L*L + Z*Z);

  if (D > L1 + L2 || D < fabsf(L1 - L2)) // 判断能否构成三角形
    return false;

  float d = atan2f(Z,L);
  float cc = (D*D + L1*L1 - L2*L2) / (2.0f*D*L1);
  cc = constrain(cc, -1.0f, 1.0f); // 防止反函数定义域超出范围
  float c = acosf(cc);

  for(int i=0;i<2;i++){
    float a1 = (i == 0) ? (d + c) : (d - c);
    float a2 = atan2f(Z - L1*sinf(a1) , L - L1*cosf(a1));
    float b1 = a1 * R2D + 90.0f;
    float b2 = 180.0f - a2 * R2D;
    float b3 = a3 * R2D;

    // 检查
    if(b1 < servoMin[1] || b1 > servoMax[1] 
      || b2 < servoMin[2] || b2 > servoMax[2]
      || b3 < servoMin[0] || b3 > servoMax[0])
      continue;

    // 写入角度
    servoAngleT[1] = (int)b1;
    servoAngleT[2] = (int)b2;
    servoAngleT[0] = (int)b3;
    return 1;
  }
  // 无结果
  return 0;
}

// 函数：舵机角度正解为坐标
void incoordinate(float base,float fArm,float rArm){
  x = (L1*cosf((fArm - 90) * D2R) + L2*cosf((180 - rArm) * D2R) + L3 + l) * cosf(base * D2R);
  y = (L1*cosf((fArm - 90) * D2R) + L2*cosf((180 - rArm) * D2R) + L3 + l) * sinf(base * D2R);
  z = H + (L1*sinf((fArm - 90) * D2R)) + (L2*sinf((180 - rArm) * D2R)) - h;
}

// 函数：任务二：A
void taskA(){
  switch(taskStep){
    case 0:
      taskTime = millis();
      servoAngleT[0] = 45;
      servoAngleT[1] = 90;
      servoAngleT[2] = 90;
      taskStep = 1;
      break;
    case 1:
      if (millis() - taskTime >= 1000){
        servoAngleT[1] = 12;
        servoAngleT[2] = 145;
        taskStep = 2;
      }
      break;

    case 2:
      if(millis() - taskTime >= 2500){
        servoAngleT[3] = 175;
        taskStep = 3;
      }
      break;
      
    case 3:
      if(millis() - taskTime >= 3500){
        servoAngleT[1] = 90;
        servoAngleT[2] = 90;
        servoAngleT[0] = 135;
        taskStep = 4;
      }
      break;

    case 4:
      if(millis() - taskTime >= 5000){
        servoAngleT[1] = 12;
        servoAngleT[2] = 145;
        taskStep = 5;
      }
      break;

    case 5:
      if(millis() - taskTime >= 6500){
        servoAngleT[3] = 90;
        taskStep = 6;
      }
      break;
      
    case 6:
      if(millis() - taskTime >= 7500){
        servoAngleT[1] = 90;
        servoAngleT[2] = 90;
        taskStep = 0;
        taskState = 'P'; // 结束
      }
      break;

    default:
      break;
  }
}

// 函数：任务二：B
void taskB(){
  switch(taskStep){
    case 0:
      taskTime = millis();
      servoAngleT[0] = 0;
      servoAngleT[1] = 90;
      servoAngleT[2] = 90;
      taskStep = 1;
      break;
    case 1:
      if (millis() - taskTime >= 1000){
        servoAngleT[1] = 12;
        servoAngleT[2] = 145;
        taskStep = 2;
      }
      break;

    case 2:
      if(millis() - taskTime >= 2500){
        servoAngleT[3] = 175;
        taskStep = 3;
      }
      break;
      
    case 3:
      if(millis() - taskTime >= 3500){
        servoAngleT[1] = 90;
        servoAngleT[2] = 90;
        servoAngleT[0] = 90;
        taskStep = 4;
      }
      break;

    case 4:
      if(millis() - taskTime >= 5000){
        servoAngleT[1] = 12;
        servoAngleT[2] = 145;
        taskStep = 5;
      }
      break;

    case 5:
      if(millis() - taskTime >= 6500){
        servoAngleT[3] = 90;
        taskStep = 6;
      }
      break;
      
    case 6:
      if(millis() - taskTime >= 7500){
        servoAngleT[1] = 90;
        servoAngleT[2] = 90;
        taskStep = 0;
        taskState = 'P'; // 结束
      }
      break;

    default:
      break;
  }
}

// 函数：任务二：C
void taskC(){
  switch(taskStep){
    case 0:
      taskTime = millis();
      servoAngleT[0] = 180;
      servoAngleT[1] = 90;
      servoAngleT[2] = 90;
      taskStep = 1;
      break;
    case 1:
      if (millis() - taskTime >= 1000){
        servoAngleT[1] = 12;
        servoAngleT[2] = 145;
        taskStep = 2;
      }
      break;

    case 2:
      if(millis() - taskTime >= 2500){
        servoAngleT[3] = 175;
        taskStep = 3;
      }
      break;
      
    case 3:
      if(millis() - taskTime >= 3500){
        servoAngleT[1] = 90;
        servoAngleT[2] = 90;
        servoAngleT[0] = 0;
        taskStep = 4;
      }
      break;

    case 4:
      if(millis() - taskTime >= 5000){
        servoAngleT[1] = 12;
        servoAngleT[2] = 145;
        taskStep = 5;
      }
      break;

    case 5:
      if(millis() - taskTime >= 6500){
        servoAngleT[3] = 90;
        taskStep = 6;
      }
      break;
      
    case 6:
      if(millis() - taskTime >= 7500){
        servoAngleT[1] = 90;
        servoAngleT[2] = 90;
        taskStep = 0;
        taskState = 'P'; // 结束
      }
      break;

    default:
      break;
  }
}

// 函数：任务三：记录模式
void startRecord(){
  pointCount = 0;
  isRecord = true;
  isPlay = false;
  lastRecordTime = millis();
}
void stopRecord(){
  isRecord = false;
}
void recordStep(){
  if(millis() - lastRecordTime >= 200){
    lastRecordTime = millis();
    
    if(pointCount < maxPoint){
      for(int i = 0; i < 4; i++){
        recordData[pointCount].angles[i] = (byte)servoAngle[i];
      }
      pointCount++;
    }else{
      isRecord = false;
    }
  }
}

// 函数：任务三：回放模式
void startPlay() {
  playCount = 0;
  isPlay = true;
  isRecord = false;
  lastRecordTime = millis();
}
void stopPlay() {
  isPlay = false;
}
void playStep() {
  if (millis() - lastRecordTime >= 200) {
    lastRecordTime = millis();
    
    if (playCount < pointCount) {
      for (int i = 0; i < 4; i++) {
        servoAngleT[i] = recordData[playCount].angles[i];
      }
      playCount++;
    } else {
      isPlay = false;
    }
  }
}

// 函数：任务五
void track() {
  if((trackNum +1) >= trackNumT){
    isTrack = false;
    trackPhase = 0;
    trackStep = 0; // 结束
    return;
  }

  float x1 = trackData[trackNum].pos[0];
  float y1 = trackData[trackNum].pos[1];
  float z1 = trackData[trackNum].pos[2];
  float x2 = trackData[trackNum+1].pos[0];
  float y2 = trackData[trackNum+1].pos[1];
  float z2 = trackData[trackNum+1].pos[2];

  if (trackPhase == 0){
    int res = coordinate(x1, y1, z1);
    if (res == 0){
      Serial.println("error.");
      isTrack = false;
      trackStep = 0; // 中断
      return;
    }
    if(millis() - trackPhaseTime >= 2000){
      trackPhase = 1;
      trackStep = 0;
    }
    return;
  } // 当trackPhase != 0 程序向下执行
  
  // 计算最大差值，决定总步数
  float maxDiff = fabsf(x2 - x1);
  if(fabsf(y2 - y1) > maxDiff)
    maxDiff = fabsf(y2 - y1);
  if(fabsf(z2 - z1) > maxDiff)
    maxDiff = fabsf(z2 - z1);
  
  int totalSteps = (int)(maxDiff * 2);
  if (totalSteps < 1) totalSteps = 1;
  
  // 每 loop 走一步
  if (trackStep <= totalSteps){
    float t = (float)trackStep / totalSteps;
    int res = coordinate(x1 + (x2 - x1) * t , y1 + (y2 - y1) * t , z1 + (z2 - z1) * t);
    if (res == 0){
      Serial.println("error.");
      isTrack = false;
      trackPhase = 0;
      trackStep = 0; // 中断
      return;
    }
    trackStep++;
  }else{
    trackStep = 0; // 结束
    trackPhaseTime = millis();
    trackNum++;
  }
}

// 主循环
void setup() {

  //设定舵机引脚
  for(int i=0;i<4;i++){
    servo[i].attach(servoPin[i]);
    delay(200);
  }

  //设定初始角度
  for(int i=0;i<4;i++){
    servo[i].write(servoAngleT[i]);
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

  // 摇杆输入到函数
  if(isPlay == false && taskState == 'P' && isTrack == false){
    servoChange(0,-joystickAct(lx));
    servoChange(1,joystickAct(ry));
    servoChange(2,-joystickAct(ly));
    servoChange(3,joystickAct(rx));
  }

  // 舵机角度执行
  for(int i=0;i<4;i++){
    if(servoAngle[i] < servoAngleT[i]){
      servoAngle[i]++; // 加速
    }else if(servoAngle[i] > servoAngleT[i]){
      servoAngle[i]--; // 减速.
    }
    servo[i].write(servoAngle[i]);
  }

  // 串口通信
  // static unsigned long lastPrintTime = 0; 
  // if (millis() - lastPrintTime >= 1000) { // 每1000毫秒执行一次
  //   reportAngles();
  //   lastPrintTime = millis(); // 更新时间戳
  // }

  // 上位机信息传递主程序
  if(Serial.available()){
    char command = Serial.read();
    if (command == '\n' || command == '\r') {
      // 静默
    } else {
      Serial.print("Command:");
      Serial.println(command);
    }

    switch(command){
      // 任务一：固定指令通信
      case 'O':
        servoAngleT[3] = 70;
        break;

      case 'S':
        servoAngleT[3] = 175;
        break;

      case 'H':
        if(delaytime > 3){
          delaytime -= 2;
        }else{
          Serial.println("Max speed!");
        }
        break;

      case 'L':
        if(delaytime < 15){
          delaytime += 2;
        }else{
          Serial.println("Min speed!");
        }
        break;
      // 任务一：多舵机协同控制
      case 'x':{
        int x = Serial.parseInt();
        x = constrain(x, servoMin[0], servoMax[0]);
        int y = Serial.parseInt();
        y = constrain(y, servoMin[1], servoMax[1]);
        int z = Serial.parseInt();
        z = constrain(z, servoMin[2], servoMax[2]);
        servoAngleT[0] = x;
        servoAngleT[1] = y;
        servoAngleT[2] = z;
        break;
      }
      // 任务二
      case 'A':{  // 将45°物体移向135°
        if(taskState == 'P'){
          taskState = 'A';
          taskStep = 0;
        }
        break;
      }

      case 'B':{  // 将0°物体移向90°
        if(taskState == 'P'){
          taskState = 'B';
          taskStep = 0;
        }
        break;
      }

      case 'C':{  // 将180°物体移向0°
        if(taskState == 'P'){
          taskState = 'C';
          taskStep = 0;
        }
        break;
      }

      // 任务五.1：直线绘制
      case 'T':
        if(!isTrack){
          trackData[0].pos[0] = -100;
          trackData[0].pos[1] = 100;
          trackData[0].pos[2] = penHeight;
          trackData[1].pos[0] = 100;
          trackData[1].pos[1] = 100;
          trackData[1].pos[2] = penHeight;
          trackStep = 0;
          trackPhase = 0;
          trackNumT = 2;
          trackNum = 0;
          trackPhaseTime = millis();
          isTrack = true;
        }
        break;

        // 任务五.2：三角形
        case 'Y':
        if(!isTrack){
          trackData[0].pos[0] = -100;
          trackData[0].pos[1] = 100;
          trackData[0].pos[2] = penHeight;
          trackData[1].pos[0] = 100;
          trackData[1].pos[1] = 100;
          trackData[1].pos[2] = penHeight;
          trackData[2].pos[0] = 0;
          trackData[2].pos[1] = 200;
          trackData[2].pos[2] = penHeight;
          trackData[3].pos[0] = -100;
          trackData[3].pos[1] = 100;
          trackData[3].pos[2] = penHeight;
          trackStep = 0;
          trackPhase = 0;
          trackNumT = 4;
          trackNum = 0;
          trackPhaseTime = millis();
          isTrack = true;
        }
        break;

      // 循环执行A、B、C（按键1）（等待电路同学完成后迁移）
      case 'V':
        if(times1 == 0 && taskState == 'P'){
            taskState = 'A';
            taskStep = 0;
            times1++;
        }else if(times1 == 1 && taskState == 'P'){
            taskState = 'B';
            taskStep = 0;
            times1++;
        }else if(times1 == 2 && taskState == 'P'){
            taskState = 'C';
            taskStep = 0;
            times1 = 0;
        }
      break;

      // 启动或结束录制（按键2）（等待电路同学完成后迁移）
      case 'N':
        if(isRecord)
          stopRecord();
        else
          startRecord();
        break;

      // 启动回放（按键3）（等待电路同学完成后迁移）
      case 'M':
        startPlay();
        break;

      // 机械臂回中（按键4）（等待电路同学完成后迁移）
      case 'Z':
        for (int i=0;i<4;i++){
          servoAngleT[i] = 90;
        }
        break;

      default:
        break;
    }
  }

  // 任务三：任务A执行
  if(taskState == 'A'){
    taskA();
  }

  // 任务三：任务B执行
  if(taskState == 'B'){
    taskB();
  }

  // 任务三：任务C执行
  if(taskState == 'C'){
    taskC();
  }

  // 任务三：录制执行
  if(isRecord)
    recordStep();

  // 任务三：回放执行
  if(isPlay)
    playStep();

  if(isTrack)
    track();

  delay(delaytime);
}