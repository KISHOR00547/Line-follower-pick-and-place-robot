#include <Servo.h>
#include <SoftwareSerial.h>

/* ---------- BLUETOOTH ---------- */
SoftwareSerial BT(-1, 1);

/* ---------------- MOTOR PINS ---------------- */
#define AIN1 8
#define AIN2 9
#define PWMA 6
#define BIN1 7
#define BIN2 4
#define PWMB 5

/* ---------------- LINE SENSORS ---------------- */
#define S0 A0
#define S1 A1
#define S2 A2
#define S3 A3
#define S4 A4
#define S5 A5

/* ---------------- IR SENSOR ---------------- */
#define IR_PIN 10

/* ---------------- TCS3200 ---------------- */
#define TCS_S2 3
#define TCS_S3 12
#define TCS_OUT 13

/* ---------------- SERVOS ---------------- */
Servo elbow;
Servo gripper;

/* ---------------- THRESHOLD ---------------- */
int th0=850, th1=850, th2=850, th3=850, th4=850, th5=850;

/* ---------------- SPEED ---------------- */
int SPEED_NORMAL = 97;
#define SPEED_AFTER_FIRST_CROSS 105   // ⭐ NEW
int SPEED_AFTER_PICK = 140;
int forwardSpeed = SPEED_NORMAL;

/* ---------------- MEMORY ---------------- */
int lastSeen=0;
unsigned long lostTime=0;

/* ---------------- STATE ---------------- */
enum State { COUNTING, PICKING, DOTTED, FINISHED };
State state=COUNTING;

/* ---------------- COLOR COUNT ---------------- */
int X=0,Y=0,Z=0;
char winner='N';
bool carrying=false;

/* ---------------- TIMERS ---------------- */
unsigned long lastColorTime=0,lastCrossTime=0,dottedStartTime=0;

/* ---------------- CROSS FILTER ---------------- */
unsigned long crossDetectTime=0;
bool crossArmed=false,crossConsumed=false;

/* ---------------- FLAGS ---------------- */
bool firstCrossPassed=false;
bool waitingForBox=false;

/* -------- FIRST CROSS STOP -------- */
bool crossPauseDone=false;
unsigned long crossStopTime=0;
#define FIRST_CROSS_STOP_TIME 500

/* ---------------- MOTOR CONTROL ---------------- */
void leftMotor(int s){
  s=constrain(s,-255,255);
  digitalWrite(AIN1,s>0);
  digitalWrite(AIN2,s<0);
  analogWrite(PWMA,abs(s));
}
void rightMotor(int s){
  s=constrain(s,-255,255);
  digitalWrite(BIN1,s>0);
  digitalWrite(BIN2,s<0);
  analogWrite(PWMB,abs(s));
}

/* ---------------- COLOR SENSOR ---------------- */
unsigned long readColor(bool s2,bool s3){
  digitalWrite(TCS_S2,s2);
  digitalWrite(TCS_S3,s3);
  delayMicroseconds(3);
  unsigned long v=pulseIn(TCS_OUT,LOW,20000);
  return v==0?99999:v;
}
bool isRed(unsigned long r,unsigned long g,unsigned long b){
  return (r>50 && r<100 && g>80 && g<170 && b>80 && b<150);
}
bool isGreen(unsigned long r,unsigned long g,unsigned long b){
  return (g<150 && g>80 && r<120 && r>80 && b>80 && b<150);
}
bool isBlue(unsigned long r,unsigned long g,unsigned long b){
  return (b>80 && b<100 && r>120 && r<150 && g>120 && g<150);
}

/* ---------------- PICK BOX ---------------- */
void pickBox(){
  BT.println("PICKING BOX");

  leftMotor(0); rightMotor(0); delay(300);

  gripper.write(20); delay(300);
  elbow.write(170); delay(600);
  gripper.write(90); delay(300);
  elbow.write(0); delay(600);

  carrying=true;
  forwardSpeed=SPEED_AFTER_PICK;

  waitingForBox=false;
  BT.println("BOX PICKED");
}

/* ---------------- PLACE BOX ---------------- */
void placeBox(){
  BT.println("PLACING BOX");

  leftMotor(0); rightMotor(0);

  elbow.write(170); delay(600);
  gripper.write(20); delay(300);
  elbow.write(0); delay(600);

  BT.println("TASK COMPLETED");
  BT.println("ROBOT STOPPED");

  while(1);
}

/* ---------------- REAL CROSS ---------------- */
bool isRealCross(){
  bool allBlack=
    (analogRead(S0)>th0 &&
     analogRead(S1)>th1 &&
     analogRead(S2)>th2 &&
     analogRead(S3)>th3 &&
     analogRead(S4)>th4 &&
     analogRead(S5)>th5);

  if(allBlack && !crossConsumed){
    if(!crossArmed){
      crossArmed=true;
      crossDetectTime=millis();
    }
    if(millis()-crossDetectTime>120){
      crossConsumed=true;
      return true;
    }
  }
  if(!allBlack){
    crossArmed=false;
    crossConsumed=false;
  }
  return false;
}

/* ---------------- PID LINE FOLLOW ---------------- */
void followLine(){
  bool b[6];
  int active=0;
  float pos=0;
  bool center=false,left=false,right=false;

  for(int i=0;i<6;i++){
    b[i]=analogRead(A0+i)>th0;
    if(b[i]){
      active++;
      pos+=(i-2.5);
      if(i==2||i==3) center=true;
      if(i<2) left=true;
      if(i>3) right=true;
    }
  }

  if(left&&!right) lastSeen=-1;
  if(right&&!left) lastSeen=1;

  if(active==0){
    if(lostTime==0) lostTime=millis();
    if(millis()-lostTime<250){
      if(lastSeen<=0) leftMotor(-60),rightMotor(60);
      else leftMotor(60),rightMotor(-60);
    }else{
      if(lastSeen<=0) leftMotor(-120),rightMotor(120);
      else leftMotor(120),rightMotor(-120);
    }
    return;
  }
  lostTime=0;

  float error=pos/active;
  if(center && abs(error)<0.4) error=0;
  else if(abs(error)<0.25) error=0;

  static float smooth=0,last=0,prev=0;
  smooth=0.88*smooth+0.12*error;
  float dErr=smooth-last; last=smooth;

  float Kp=center?10.5:8.0;
  float Kd=12.0;
  float corr=Kp*smooth+Kd*dErr;
  corr=constrain(corr,-55,55);

  float step=corr-prev;
  if(step>3) corr=prev+3;
  if(step<-3) corr=prev-3;
  prev=corr;

  int L=forwardSpeed-corr;
  int R=forwardSpeed+corr;

  if(center){
    L=constrain(L,75,190);
    R=constrain(R,75,190);
  }

  if(!center && abs(error)>2.2){
    if(error>0) leftMotor(145),rightMotor(-145);
    else leftMotor(-145),rightMotor(145);
    return;
  }

  leftMotor(L);
  rightMotor(R);
}

/* ---------------- SETUP ---------------- */
void setup(){
  pinMode(AIN1,OUTPUT); pinMode(AIN2,OUTPUT); pinMode(PWMA,OUTPUT);
  pinMode(BIN1,OUTPUT); pinMode(BIN2,OUTPUT); pinMode(PWMB,OUTPUT);
  pinMode(IR_PIN,INPUT);
  pinMode(TCS_S2,OUTPUT); pinMode(TCS_S3,OUTPUT); pinMode(TCS_OUT,INPUT);

  elbow.attach(2);
  gripper.attach(11);
  elbow.write(0); gripper.write(90);

  BT.begin(115200);
  BT.println("SYSTEM STARTED");
}

/* ---------------- LOOP ---------------- */
void loop(){

/* -------- COUNT COLORS -------- */
if(state==COUNTING && millis()-lastColorTime>600){
  unsigned long r=readColor(LOW,LOW);
  unsigned long g=readColor(HIGH,HIGH);
  unsigned long b=readColor(LOW,HIGH);

  if(isRed(r,g,b)) X++;
  else if(isGreen(r,g,b)) Y++;
  else if(isBlue(r,g,b)) Z++;

  lastColorTime=millis();
}

/* -------- FIRST CROSS -------- */
if(!firstCrossPassed && millis()-lastCrossTime>800 && isRealCross()){
  lastCrossTime=millis();

  winner=(X>=Y && X>=Z)?'R':(Y>=X && Y>=Z)?'G':'B';

  state=PICKING;
  firstCrossPassed=true;
  waitingForBox=true;

  // ⭐ SPEED BOOST AFTER FIRST CROSS
  forwardSpeed = SPEED_AFTER_FIRST_CROSS;

  leftMotor(0); rightMotor(0);
  crossStopTime=millis();
  crossPauseDone=false;

  BT.println("FIRST CROSS");
  BT.println("SPEED BOOST TO 105");
}

/* -------- STOP AT FIRST CROSS -------- */
if(firstCrossPassed && !crossPauseDone){
  if(millis()-crossStopTime < FIRST_CROSS_STOP_TIME){
    leftMotor(0);
    rightMotor(0);
    return;
  }else{
    crossPauseDone=true;
  }
}

/* -------- WAIT FOR IR + COLOR -------- */
if(state==PICKING && waitingForBox){
  if(digitalRead(IR_PIN)==LOW){

    unsigned long r=readColor(LOW,LOW);
    unsigned long g=readColor(HIGH,HIGH);
    unsigned long b=readColor(LOW,HIGH);

    char cur=isRed(r,g,b)?'R':
             isGreen(r,g,b)?'G':
             isBlue(r,g,b)?'B':'N';

    if(cur==winner){
      pickBox();
    }
  }
}

/* -------- AFTER PICK -------- */
if(state==PICKING && carrying){
  int s=0;
  for(int i=0;i<6;i++) if(analogRead(A0+i)>th0) s++;

  if(s<=1){
    if(dottedStartTime==0) dottedStartTime=millis();
    if(millis()-dottedStartTime>250){
      state=DOTTED;
      BT.println("DOTTED ZONE");
    }
  }else dottedStartTime=0;
}

/* -------- SECOND CROSS -------- */
if(state==DOTTED && isRealCross()){
  placeBox();
}

/* -------- LINE FOLLOW -------- */
followLine();

}
