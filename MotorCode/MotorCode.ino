
#include <Pixy2.h>

#define PWM1 2
#define AIN1 26
#define AIN2 24
#define PWM2 3
#define BIN1 30
#define BIN2 32
#define STANDBY 28

#define TRIG_L 20 // Left Ultrasonic
#define ECHO_L 22 
#define TRIG_R 31 // Right ultrasonic
#define ECHO_R 29
#define ECHO_B 49 // Back ultrasonic
#define TRIG_B 47

#define BR_S 43 // Back right sensor
#define FL_S 23 // Fron left sensor
#define BL_S 45 // Back left sensor
#define FR_S 25 // Front right sensor

Pixy2 pixy;
void setupPixy();
Block getPixyBlockData();
uint16_t getPixyXCoord();
uint16_t getPixyColor();
uint16_t x;

//long duration, cm, dist;

void setup() {
  Serial.begin(9600);
  pinMode(PWM1,OUTPUT);
  pinMode(AIN1,OUTPUT);
  pinMode(AIN2,OUTPUT);
  pinMode(PWM2,OUTPUT);
  pinMode(BIN1,OUTPUT);
  pinMode(BIN2,OUTPUT);
  
  pinMode(BR_S, INPUT); 
  pinMode(BL_S, INPUT);
  pinMode(FR_S, INPUT); 
  pinMode(FL_S, INPUT);
  
  pinMode(TRIG_L,OUTPUT);
  pinMode(ECHO_L,INPUT);
  pinMode(TRIG_R,OUTPUT);
  pinMode(ECHO_R,INPUT);
  pinMode(TRIG_B,OUTPUT);
  pinMode(ECHO_B,INPUT);
  
  digitalWrite(STANDBY,HIGH);

  setupPixy();
}

// reading 1 is white space
bool isBR_S_Active = false;
bool isBL_S_Active = false;
bool isFR_S_Active = false;
bool isFL_S_Active = false;

void loop() {
  isBR_S_Active = digitalRead(BR_S) == 0;
  isBL_S_Active = digitalRead(BL_S) == 0;
  isFR_S_Active = digitalRead(FR_S) == 0;
  isFL_S_Active = digitalRead(FL_S) == 0;

//Serial.println(isBR_S_Active);
//Serial.println(isBL_S_Active);
//Serial.println(isFR_S_Active);
//Serial.println(isFL_S_Active);


//if (!isFL_S_Active && !isFR_S_Active && !isBL_S_Active && !isBR_S_Active) {
//    forward(); // Move forward when all sensors are inactive
//} else if (isFR_S_Active || isFL_S_Active) {
//    backward(); // Move backward if any of the front sensors are active
//} else if (isBR_S_Active) {
//    turnLeft(); // Turn left if back right sensor active
//} else if (isBL_S_Active) {
//    turnRight(); // Turn right if back left sensor active
//} else {
//    forward(); // Move forward if none of the above conditions are met
//}

//if (isFR_S_Active) {
//    backward(); // Move forward when all sensors are inactive
//} else if (isFL_S_Active) {
//    backward(); // Move backward if any of the front sensors are active
//} else if (isBR_S_Active) {
//    turnLeft(); // Turn left if back right sensor active
//} else if (isBL_S_Active) {
//    turnRight(); // Turn right if back left sensor active
//} else {
//    forward(); // Move forward if none of the above conditions are met
//}


  // CHANGE THESE VALUES LATER 
  Block pixyBlocks = getPixyBlockData();
  x = getPixyXCoord();
  bool targetIsOnTheLeft = x < 145; // 0-315
  bool targetIsOnTheRight = x > 170;
  bool shoot = x > 145 && x < 170;
  
  if (targetIsOnTheLeft) {
    Serial.println("left");
    turnLeft();
  }
  else if (targetIsOnTheRight) {
    Serial.println("right");
    turnRight();
  }
  else if (shoot){
    // turn on servo and shoot
    Serial.println("shoot");
    stopMotor();
    delay(5000);
  }

//  delay(500);
}

//void loop() {
//  dist = getUltrasonicDist();
//  if (dist <= 10) {
//    stopMotor();
//  }
//  else {
//    forward();
//  }
//  delay(250);
//}

void stopMotor(){
  digitalWrite(AIN1,LOW); //Motor A Rotate Counter Clockwise
  digitalWrite(AIN2,LOW);
  digitalWrite(BIN1,LOW); //Motor B Rotate Counter Clockwise
  digitalWrite(BIN2,LOW);
  analogWrite(PWM1,0);
  analogWrite(PWM2,0);
}

/* need to callibrate the direction and PWM value */
void forward(){
  digitalWrite(AIN1,LOW); //Motor A Rotate Counter Clockwise
  digitalWrite(AIN2,HIGH);
  digitalWrite(BIN1,LOW); //Motor B Rotate Counter Clockwise
  digitalWrite(BIN2,HIGH);
  analogWrite(PWM1,200);
  analogWrite(PWM2,200);
}

void backward(){
  digitalWrite(AIN1,HIGH);
  digitalWrite(AIN2,LOW);
  digitalWrite(BIN1,HIGH);
  digitalWrite(BIN2,LOW);
  analogWrite(PWM1,100);
  analogWrite(PWM2,100);
}

void turnRight() {
  digitalWrite(AIN1,HIGH); // 1 = left motor, 2 = right motor, both HIGH = forward, both LOW = backward
  digitalWrite(AIN2,LOW);
  digitalWrite(BIN1,HIGH);  
  digitalWrite(BIN2,LOW);
  analogWrite(PWM1,100);
  analogWrite(PWM2,100);
}

void turnLeft() {
  digitalWrite(AIN1,LOW);
  digitalWrite(AIN2,HIGH);
  digitalWrite(BIN1,LOW);
  digitalWrite(BIN2,HIGH);
  analogWrite(PWM1,100);
  analogWrite(PWM2,100);
}

void setupPixy() {
  pixy.init();
}

Block getPixyBlockData(){ 
  pixy.ccc.getBlocks();
  if (pixy.ccc.numBlocks){
    Serial.print("Detected ");
    Serial.println(pixy.ccc.numBlocks);
    pixy.ccc.blocks[0].print();
    return pixy.ccc.blocks[0];
  }  
}

uint16_t getPixyXCoord() {
  return getPixyBlockData().m_x;
}

uint16_t getPixyColor() {
  return getPixyBlockData().m_signature;
}

///* get distance from ultrasonic sensor */
//  long getUltrasonicDist(){
//  digitalWrite(TRIG, LOW);
//  delayMicroseconds(5);
//  digitalWrite(TRIG, HIGH);
//  delayMicroseconds(10);
//  digitalWrite(TRIG, LOW);
//  duration = pulseIn(ECHO, HIGH);
//  cm = (duration/2) / 29.1;
//  Serial.print(cm);
//  Serial.println("cm");
//  return cm;
//}
