#include <Pixy2I2C.h>
#include <Servo.h>

#define PWM1 2
#define AIN1 26
#define AIN2 24
#define PWM2 3
#define BIN1 30
#define BIN2 32
#define STANDBY 28

#define SERVO 9

#define TRIG_L 20 // Left Ultrasonic
#define ECHO_L 22
#define TRIG_R 31 // Right ultrasonic
#define ECHO_R 29
#define ECHO_B 49 // Back ultrasonic
#define TRIG_B 47

#define BR_S 50 // Back right sensor
#define FL_S 23 // Front left sensor
#define BL_S 45 // Back left sensor
#define FR_S 25 // Front right sensor

#define NO_TARGET_DETECTED 999
#define PINK_COLOR 0b00000011
#define GREEN_COLOR 0b00000100

Pixy2I2C pixy;
Servo myservo;

bool targetIsOnTheLeft = false;
bool targetIsOnTheRight = false;
bool targetIsAtTheMiddle = false;
bool isTurningLeft = false;
bool isTurningRight = false;
const int p = 1; // p value for motor speed
int turning_speed = 0; // motor turning speed when it sees enemy robot
const int middle = 158; // middle of pixy view

int ballCounter = 10;
unsigned long time;
int randDelay;
int randDir;

// reading 1 is white space
bool isBR_S_Active = false;
bool isBL_S_Active = false;
bool isFR_S_Active = false;
bool isFL_S_Active = false;

//long duration, cm, dist;

void setup() {
  Serial.begin(9600);
  pinMode(PWM1, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(BR_S, INPUT);
  pinMode(BL_S, INPUT);
  pinMode(FR_S, INPUT);
  pinMode(FL_S, INPUT);

  pinMode(TRIG_L, OUTPUT);
  pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT);
  pinMode(ECHO_R, INPUT);
  pinMode(TRIG_B, OUTPUT);
  pinMode(ECHO_B, INPUT);

  digitalWrite(STANDBY, HIGH);

  myservo.attach(SERVO);  // attaches the servo on pin 9 to the servo object

  setupPixy();
  randomSeed(0);
}

void loop() {
  time = millis();
  getIRData();
  if (ballCounter > 0) {
    defend();
  }
  else {
    reload();
  }
}


void defend() {
  uint16_t pinkTarget; // x location of enemy robot
  bool noPinkTarget  = false;

  /* AIMING DATA FROM PIXY */
  pinkTarget = getPinkCoord();
  // update the target data
  if (pinkTarget == NO_TARGET_DETECTED) {
    noPinkTarget  = true;
    targetIsOnTheLeft = false;
    targetIsOnTheRight = false;
    targetIsAtTheMiddle = false;
  }
  else {
    noPinkTarget  = false;
    targetIsOnTheLeft = pinkTarget < (middle - 10);
    targetIsOnTheRight = pinkTarget > (middle + 10);
    targetIsAtTheMiddle = pinkTarget >= (middle - 10) && pinkTarget <= (middle + 10);
  }

  randDelay = random(750, 2000);
  randDir = random(2);
  ///////////////// IR SENSOR /////////////////////////////
  if (isFR_S_Active) {
    backward(100);
    while (millis() < time + randDelay) {
      if (randDir == 1) {
        turnLeftARW(100);
      }
      else {
        turnLeftFW(100);
      }
      getIRData();
      if (isFL_S_Active || isFR_S_Active || isBL_S_Active || isBR_S_Active) {
        break;
      }
      if (getPinkCoord() != NO_TARGET_DETECTED) {
        break;
      }
    }
  }
  else if (isFL_S_Active) {
    backward(100);
    while (millis() < time + randDelay) {
      if (randDir == 1) {
        turnRightALW(100);
      }
      else {
        turnRightFW(100);
      }
      getIRData();
      if (isFL_S_Active || isFR_S_Active || isBL_S_Active || isBR_S_Active) {
        break;
      }
      if (getPinkCoord() != NO_TARGET_DETECTED) {
        stopMotor();
        break;
      }
    }
  }
  else if (isBR_S_Active) {
    forward(100);
    while (millis() < time + randDelay) {
      if (randDir == 1) {
        turnLeftARW(100);
      }
      else {
        turnLeftFW(100);
      }
      getIRData();
      if (isFL_S_Active || isFR_S_Active || isBL_S_Active || isBR_S_Active) {
        break;
      }
      if (getPinkCoord() != NO_TARGET_DETECTED) {
        stopMotor();
        break;
      }
    }
  }
  else if (isBL_S_Active) {
    forward(100);
    while (millis() < time + randDelay) {
      if (randDir == 1) {
        turnRightALW(100);
      }
      else {
        turnRightFW(100);
      }
      getIRData();
      if (isFL_S_Active || isFR_S_Active || isBL_S_Active || isBR_S_Active) {
        break;
      }
      if (getPinkCoord() != NO_TARGET_DETECTED) {
        stopMotor();
        break;
      }
    }
  }
  if (!isFL_S_Active && !isFR_S_Active && !isBL_S_Active && !isBR_S_Active) {
    /////////////////////// PIXY CONTROL //////////////////////////
    if (noPinkTarget) {
      forward(100);
      Serial.println("FW");
    }
    else {
      /* Aiming the target */
      ///// MOTOR SPEED P CONTROL //////////
      turning_speed = p * (middle - pinkTarget);
      turning_speed = map(abs(turning_speed), 0, 158, 10, 125); // changing turning speed to pwm control (puts value between 10 and 200 out of 255)
      Serial.print("turning speed = ");
      Serial.println(turning_speed);

      if (targetIsOnTheLeft) {
        Serial.println("left");
        isTurningLeft = true;
        isTurningRight = false;
        turnLeft(turning_speed);
      }
      else if (targetIsOnTheRight) {
        Serial.println("right");
        isTurningLeft = false;
        isTurningRight = true;
        turnRight(turning_speed);
      }
      else if (targetIsAtTheMiddle) {
        stopMotor();
        shoot();
      }
    }
  }
}

void reload() {
  uint16_t greenTarget; // x location of the reloading location
  bool isAtReloadStation = false; // !!!!! UPDATE THIS (how to tell that it reaches the station so that it stops) !!!!!
  bool noGreenTarget = false;

  greenTarget = getGreenCoord();
  // update the station data
  if (greenTarget == NO_TARGET_DETECTED) {
    noGreenTarget  = true;
    targetIsOnTheLeft = false;
    targetIsOnTheRight = false;
    targetIsAtTheMiddle = false;
  }
  else {
    noGreenTarget  = false;
    targetIsOnTheLeft = greenTarget < (middle - 10); // 0-315
    targetIsOnTheRight = greenTarget > (middle + 10);
    targetIsAtTheMiddle = greenTarget >= (middle - 10) && greenTarget <= (middle + 10);
  }

  randDelay = 100;
  if (isFR_S_Active) {
    backward(100);
    while (millis() < time + randDelay) {
      turnLeftARW(100);
      getIRData();
      if (isFL_S_Active || isFR_S_Active || isBL_S_Active || isBR_S_Active) {
        break;
      }
    }
  }
  else if (isFL_S_Active) {
    backward(100);
    while (millis() < time + randDelay) {
      turnRightALW(100);
      getIRData();
      if (isFL_S_Active || isFR_S_Active || isBL_S_Active || isBR_S_Active) {
        break;
      }
    }
  }
  else if (isBR_S_Active) {
    forward(100);
    while (millis() < time + randDelay) {
      turnLeftARW(100);
      getIRData();
      if (isFL_S_Active || isFR_S_Active || isBL_S_Active || isBR_S_Active) {
        break;
      }
    }
  }
  else if (isBL_S_Active) {
    forward(100);
    while (millis() < time + randDelay) {
      turnRightALW(100);
      getIRData();
      if (isFL_S_Active || isFR_S_Active || isBL_S_Active || isBR_S_Active) {
        break;
      }
    }
  }
  if (!isFL_S_Active && !isFR_S_Active && !isBL_S_Active && !isBR_S_Active) {
    if (noGreenTarget) { // if doesn't see any green
      turnLeft(100);
      forward(100);
    }
    else {
      /* Aiming the target */
      turning_speed = p * (middle - greenTarget);
      turning_speed = map(abs(turning_speed), 0, 158, 10, 125); // changing turning speed to pwm control (puts value between 10 and 200 out of 255)
      Serial.print("turning speed = ");
      Serial.println(turning_speed);

      if (isAtReloadStation) { // always false (for now)
        // not it just run forward the green target (no stop)
        // reload
        delay(5000);
        ballCounter = 10;
      }
      else if (targetIsOnTheRight) {
        isTurningLeft = false;
        isTurningRight = true;
        turnRight(turning_speed);
      }
      else if (targetIsOnTheLeft) {
        isTurningLeft = true;
        isTurningRight = false;
        turnLeft(turning_speed);
      }
      else if (targetIsAtTheMiddle) {
        forward(130);
      }
    }
  }
}

void getIRData() {
  isBR_S_Active = digitalRead(BR_S) == 0;
  isBL_S_Active = digitalRead(BL_S) == 0;
  isFR_S_Active = digitalRead(FR_S) == 0;
  isFL_S_Active = digitalRead(FL_S) == 0;
}

void stopMotor() {
  digitalWrite(AIN1, LOW); //Motor A Rotate Counter Clockwise
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); //Motor B Rotate Counter Clockwise
  digitalWrite(BIN2, LOW);
  analogWrite(PWM1, 0);
  analogWrite(PWM2, 0);
}

/* need to callibrate the direction and PWM value */
void forward(int w) {
  digitalWrite(AIN1, LOW); //Motor A Rotate Counter Clockwise
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW); //Motor B Rotate Counter Clockwise
  digitalWrite(BIN2, HIGH);
  analogWrite(PWM1, w);
  analogWrite(PWM2, w);
}

void backward(int w) {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWM1, w);
  analogWrite(PWM2, w);
}

void turnLeftARW(int w) { // turn left about right wheel
  digitalWrite(AIN1, LOW); // B = left motor, A = right motor, both HIGH = forward, both LOW = backward
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWM1, 0);
  analogWrite(PWM2, w);
}

void turnLeftFW(int w) { // turn left forward (for back wheels)
  digitalWrite(AIN1, LOW); // B = left motor, A = right motor, both HIGH = forward, both LOW = backward
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWM1, w);
  analogWrite(PWM2, 0);
}

void turnRightALW(int w) {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWM1, w);
  analogWrite(PWM2, 0);
}

void turnRightFW(int w) {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWM1, 0);
  analogWrite(PWM2, w);
}

void turnLeft(int w) {
  digitalWrite(AIN1, LOW); // 1 = left motor, 2 = right motor, both HIGH = forward, both LOW = backward
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWM1, w);
  analogWrite(PWM2, w);
}

void turnRight(int w) {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWM1, w);
  analogWrite(PWM2, w);
}

void shoot() {
  // change the angle for next time through the loop:
  myservo.writeMicroseconds(750);
  delay(100);
  myservo.writeMicroseconds(2000);
  delay(100);
  ballCounter -= 1;
  Serial.print("Balls Left :");
  Serial.println(ballCounter);
}

void setupPixy() {
  pixy.init();
}

Block getPixyBlockData() {
  pixy.ccc.getBlocks();
  if (pixy.ccc.numBlocks) {
    //Serial.print("Detected ");
    //Serial.println(pixy.ccc.numBlocks);
    //    pixy.ccc.blocks[0].print();
    return pixy.ccc.blocks[0];

  }
}

uint16_t getPinkCoord() {
  pixy.ccc.getBlocks(PINK_COLOR);
  if (pixy.ccc.numBlocks) {
    return pixy.ccc.blocks[0].m_x;
  }
  return NO_TARGET_DETECTED;
}

uint16_t getGreenCoord() {
  pixy.ccc.getBlocks(GREEN_COLOR);
  if (pixy.ccc.numBlocks) {
    return pixy.ccc.blocks[0].m_x;
  }
  return NO_TARGET_DETECTED;
}

uint16_t getPixyXCoord() {
  pixy.ccc.getBlocks();
  if (pixy.ccc.numBlocks) {
    return pixy.ccc.blocks[0].m_x;
  }
  return NO_TARGET_DETECTED;
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
