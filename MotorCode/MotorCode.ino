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

#define BR_S 43 // Back right sensor
#define FL_S 44 // Front left sensor
#define BL_S 45 // Back left sensor
#define FR_S 42 // Front right sensor

#define NO_TARGET_DETECTED 999

Pixy2I2C pixy;
void setupPixy();
Block getPixyBlockData();
uint16_t getPixyXCoord();
uint16_t getPixyColor();
uint16_t x; // x location of enemy robot
unsigned int pause = 25;
bool isTurningLeft = false;
bool isTurningRight = false;
const int p = 1; // p value for motor speed
int turning_speed = 0; // motor turning speed when it sees enemy robot
const int middle = 158; // middle of pixy view
int ballCounter = 10;

Servo myservo;

unsigned long time;
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

// reading 1 is white space
bool isBR_S_Active = false;
bool isBL_S_Active = false;
bool isFR_S_Active = false;
bool isFL_S_Active = false;

bool targetIsOnTheLeft = false;
bool targetIsOnTheRight = false;
bool shoot = false;
bool noTarget = false;


void loop() {
  time = millis();
  //getIRData();
  isBR_S_Active = digitalRead(BR_S) == 0;
  isBL_S_Active = digitalRead(BL_S) == 0;
  isFR_S_Active = digitalRead(FR_S) == 0;
  isFL_S_Active = digitalRead(FL_S) == 0;
  //  Serial.println("IR sensor data :");
  Serial.println(isBR_S_Active);
  Serial.println(isBL_S_Active);
  Serial.println(isFR_S_Active);
  Serial.println(isFL_S_Active);

  /* AIMING DATA FROM PIXY */
  x = getPixyXCoord();
  // update the target data
  if (x == NO_TARGET_DETECTED) {
    noTarget = true;
    targetIsOnTheLeft = false;
    targetIsOnTheRight = false;
    shoot = false;
  }
  else {
    noTarget = false;
    targetIsOnTheLeft = x < (middle - 10); // 0-315
    targetIsOnTheRight = x > (middle + 10);
    shoot = x >= (middle - 10) && x <= (middle + 10);
  }
  //  Serial.println("Target :");
  //  Serial.println(targetIsOnTheLeft);
  //  Serial.println(targetIsOnTheRight);
  //  Serial.println(shoot);
  //  Serial.println(noTarget);

  int randDelay = random(750, 2000);
  int randDelayFw = random(1000, 3000);
  int randDir = random(2);
  ///////////////// IR SENSOR /////////////////////////////
  if ((isFR_S_Active && isFL_S_Active) || (isBR_S_Active && isBL_S_Active)) {
    if (isBR_S_Active && isBL_S_Active) {
      while (millis() < time + 750) {
        forward(100);
        getIRData();
        if (isFL_S_Active || isFR_S_Active) {
          break;
        }
        if (getPixyXCoord() != NO_TARGET_DETECTED) {
          stopMotor();
          break;
        }
      }
    }
    else if (isFR_S_Active && isFL_S_Active) {
      while (millis() < time + 750) {
        backward(100);
        getIRData();
        if (isBL_S_Active || isBR_S_Active) {
          break;
        }
        if (getPixyXCoord() != NO_TARGET_DETECTED) {
          stopMotor();
          break;
        }
      }
    }
    while (millis() < time + randDelay) {
      if (randDir == 1) {
        turnLeft(100);                      // Move backward if any of the front sensors are active
      }
      else {
        turnRight(100);
      }
      getIRData();
      if (isFL_S_Active || isFR_S_Active || isBL_S_Active || isBR_S_Active) {
        break;
      }
      if (getPixyXCoord() != NO_TARGET_DETECTED) {
        stopMotor();
        break;
      }
    }
  }
  else if (isFR_S_Active) {
    backward(100);
    while (millis() < time + randDelay) {
      if (randDir == 1) {
        turnLeftARW(100);                      // Move backward if any of the front sensors are active
      }
      else {
        turnLeftFW(100);
      }
      Serial.println("BW-L");
      //      turnLeft(100);
      getIRData();
      if (isFL_S_Active || isFR_S_Active || isBL_S_Active || isBR_S_Active) {
        //          stopMotor();
        break;
      }
      if (getPixyXCoord() != NO_TARGET_DETECTED) {
        stopMotor();
        break;
      }
    }
  }
  else if (isFL_S_Active) {
    backward(100);
    while (millis() < time + randDelay) {
      if (randDir == 1) {
        turnRightALW(100);                        // Move backward if any of the front sensors are active
      }
      else {
        turnRightFW(100);
      }
      Serial.println("BW-R");
      //      turnRight(100);
      getIRData();
      if (isFL_S_Active || isFR_S_Active || isBL_S_Active || isBR_S_Active) {
        //          stopMotor();
        break;
      }
      if (getPixyXCoord() != NO_TARGET_DETECTED) {
        stopMotor();
        break;
      }
    }
  }
  else if (isBR_S_Active) {
    forward(100);
    //    turnLeft(100);                        // Turn left if back right sensor active
    Serial.println("FW");
    while (millis() < time + randDelay) {
      if (randDir == 1) {
        turnLeftARW(100);                      // Move backward if any of the front sensors are active
      }
      else {
        turnLeftFW(100);
      }
      getIRData();
      if (isFL_S_Active || isFR_S_Active || isBL_S_Active || isBR_S_Active) {
        //          stopMotor();
        break;
      }
      if (getPixyXCoord() != NO_TARGET_DETECTED) {
        stopMotor();
        break;
      }
    }
  }
  else if (isBL_S_Active) {
    forward(100);
    //    turnRight(100);                       // Turn right if back left sensor active
    Serial.println("FW");
    while (millis() < time + randDelay) {
      if (randDir == 1) {
        turnRightALW(100);                        // Move backward if any of the front sensors are active
      }
      else {
        turnRightFW(100);
      }
      getIRData();
      if (isFL_S_Active || isFR_S_Active || isBL_S_Active || isBR_S_Active) {
        //          stopMotor();
        break;
      }
      if (getPixyXCoord() != NO_TARGET_DETECTED) {
        stopMotor();
        break;
      }
    }
  }
  if (!isFL_S_Active && !isFR_S_Active && !isBL_S_Active && !isBR_S_Active) {
    /////////////////////// PIXY CONTROL //////////////////////////
    if (noTarget) {
      while (millis() < time + randDelayFw) {
        forward(100);
        getIRData();
        if (isFL_S_Active || isFR_S_Active || isBL_S_Active || isBR_S_Active) {
          break;
        }
        if (getPixyXCoord() != NO_TARGET_DETECTED) {
          stopMotor();
          break;
        }
      }
      while (millis() < time + randDelay) {
      if (randDir == 1) {
        turnRight(100);                        // Move backward if any of the front sensors are active
      }
      else {
        turnLeft(100);
      }
      getIRData();
      if (isFL_S_Active || isFR_S_Active || isBL_S_Active || isBR_S_Active) {
        //          stopMotor();
        break;
      }
      if (getPixyXCoord() != NO_TARGET_DETECTED) {
        stopMotor();
        break;
      }
    }
    }
    else {
      /* Aiming the target */
      ///// MOTOR SPEED P CONTROL //////////
      turning_speed = p * (middle - x);
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
      else if (shoot) {
        stopMotor();
        shootfcn();
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
void turnRight(int w) {
  digitalWrite(AIN1, LOW); //Motor A Rotate Counter Clockwise
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW); //Motor B Rotate Counter Clockwise
  digitalWrite(BIN2, HIGH);
  analogWrite(PWM1, w);
  analogWrite(PWM2, w);
}

void turnLeft(int w) {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWM1, w);
  analogWrite(PWM2, w);
}

void turnLeftARW(int w) {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWM1, 0);
  analogWrite(PWM2, w);
}

void turnLeftFW(int w) {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWM1, w);
  analogWrite(PWM2, 0);
}

void turnRightALW(int w) {
  digitalWrite(AIN1, LOW); //Motor A Rotate Counter Clockwise
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW); //Motor B Rotate Counter Clockwise
  digitalWrite(BIN2, HIGH);
  analogWrite(PWM1, w);
  analogWrite(PWM2, 0);
}

void turnRightFW(int w) {
  digitalWrite(AIN1, LOW); //Motor A Rotate Counter Clockwise
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW); //Motor B Rotate Counter Clockwise
  digitalWrite(BIN2, HIGH);
  analogWrite(PWM1, 0);
  analogWrite(PWM2, w);
}

void forward(int w) {
  digitalWrite(AIN1, LOW); // 1 = left motor, 2 = right motor, both HIGH = forward, both LOW = backward
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWM1, w);
  analogWrite(PWM2, w);
}

void backward(int w) {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWM1, w);
  analogWrite(PWM2, w);
}

void shootfcn() {
  Serial.println(myservo.read());
  // change the angle for next time through the loop:
  myservo.writeMicroseconds(750);
  delay(100);
  Serial.println(myservo.read());
  myservo.writeMicroseconds(2000);
  delay(100);
  Serial.println(myservo.read());
  ballCounter -= 1;
  Serial.println(ballCounter);
  if (ballCounter == 0) {
    // reload
    ballCounter = 10;
  }
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


// No Target

//      int randDir = random(3);                            // 0 - left or 1 - right
//      int randDelay = random(500, 1500);
//      if (randDir == 0){
//        turnLeft(75);
//      }
//      else if (randDir == 1) {
//        turnRight(75);
//      }
//      else if (randDir == 2) {
//        forward(75);
//      }
//      //delay(randDelay);
//      while (millis() < time + randDelay) {
//        if (isFL_S_Active || isFR_S_Active || isBL_S_Active || isBR_S_Active) {
//          break;
//        }
//        if (getPixyXCoord() != NO_TARGET_DETECTED) {
//            break;
//    }
//      }
//stopMotor();
