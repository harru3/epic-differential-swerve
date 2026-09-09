#include <PS4Controller.h>
#include "BluetoothSerial.h"
#include <elapsedMillis.h>

//song files
//#include "ariaMath.h"
#include "ariaMath2.h"

elapsedMillis musicTimer1;
elapsedMillis musicTimer2;
int intensity = 20;


int maxVel = 1400;
//A SIDE
#define A0_pw0 A0   //right blue
#define A0_pw1 A1   //left white
#define A0_enc0 A2  // blue
#define A0_enc1 A3  // white

#define A1_pw0 SCK   //right
#define A1_pw1 MOSI  //leftx
#define A1_enc0 TX   //blue
#define A1_enc1 RX   //white

//B SIDE
#define B0_pw0 13   //right
#define B0_pw1 12   //left
#define B0_enc0 27  // blue
#define B0_enc1 33  // white

#define B1_pw0 32   //right
#define B1_pw1 14  //leftx
#define B1_enc0 SCL   //blue
#define B1_enc1 SDA  //white

BluetoothSerial SerialBT;

// int rstick = 0;
// int lstick = 0;

elapsedMillis loopTimer;
float angleTargetA = 0.0;
float angleTargetB = 0.0;

// int vel = 0;
// elapsedMillis velTimer;


class Motor {
private:
  int pwm0;
  int pwm1;
  int enc0;
  int enc1;

  volatile long encoderCount;
  long lastEncoderCount;

  elapsedMicros lastVelRead;
  int lastPower = 0;

  elapsedMicros pidTimer;

public:
  //constructor
  Motor(int pwm0, int pwm1, int enc0, int enc1) {
    this->pwm0 = pwm0;
    this->pwm1 = pwm1;
    this->enc0 = enc0;
    this->enc1 = enc1;
    this->encoderCount = 0;
    this->lastEncoderCount = 0;
    //this->lastTime = 0;
  }

  void begin() {
    pinMode(enc0, INPUT_PULLUP);
    pinMode(enc1, INPUT_PULLUP);
    ledcAttach(pwm0, 20000, 8);
    ledcAttach(pwm1, 20000, 8);
  }

  //setting speed
  void setPower(double power) {
    //Serial.print(power);
    power = constrain(power, -255, 255);

    if (power < 10 && power > -10) {
      ledcWrite(pwm0, 0);
      ledcWrite(pwm1, 0);

    } else if (power > 0) {
      if (lastPower < 0) {  // delay to protect bridge from shorts
        ledcWrite(pwm1, 0);
        delay(1);
        ledcWrite(pwm0, (int)abs(power));
      } else {
        ledcWrite(pwm0, (int)abs(power));
      }
    } else if (power < 0) {
      if (lastPower > 0) {  // only delay on direction change
        ledcWrite(pwm0, 0);
        delay(1);
        ledcWrite(pwm1, (int)abs(power));
      } else {
        ledcWrite(pwm1, (int)abs(power));
      }
    }
    lastPower = power;
  }

  //update encoder readings
  void updateEncoder() {
    bool pinA = digitalRead(enc0);
    bool pinB = digitalRead(enc1);

    if (pinA == pinB) {
      encoderCount++;
    } else {
      encoderCount--;
    }
  }

  int getEncoderCount() {
    return encoderCount;
  }

  void resetEncoder() {
    encoderCount = 0;
  }

  float getVelocity() {
    float dt = (lastVelRead) / 1e6f;  // convert to seconds

    // Serial.print("dt: ");
    // Serial.print(dt);
    // Serial.print(" and the amount of couints inbetween: ");


    if (dt <= 0) return 0;

    long currentCount = encoderCount;
    // Serial.print(currentCount - lastEncoderCount);
    // Serial.print("  ");
    float velocity = (currentCount - lastEncoderCount) / dt;

    lastEncoderCount = currentCount;
    lastVelRead = 0;

    return velocity;
  }
};

class wheelController {
private:
  // References to physical motor objects
  Motor &motorLeft;
  Motor &motorRight;

  // Independent timers for each PID calculation loop
  elapsedMicros velocityTimer;
  elapsedMicros angleTimer;

  // Tracking variables for Integral errors to prevent windup
  float velocityIntegral = 0;
  float angleIntegral = 0;
  float lastVelocityError = 0;
  float lastAngleError = 0;

  float Kp_vel = 0.25;
  float Ki_vel = 0.0;
  float Kd_vel = 0.0;
  float Kf_vel = 0.36;

  float Kp_ang = 8.0;
  float Ki_ang = 0.0;
  float Kd_ang = 0.25;  //.25
  float Kf_ang = 0.0;

  float lastAngleTarget;

public:
  // Constructor grabs the real deal motors by reference using '&'
  wheelController(Motor &left, Motor &right)
    : motorLeft(left), motorRight(right) {}

  void update(float targetVelocity, float currentVelocity, float targetAngle, float currentAngle) {
    // 1. Calculate Velocity PID Output
    float dtVel = velocityTimer / 1000000.0;  // Convert to seconds
    velocityTimer = 0;
    float velOutput = calculateVelocityPID(targetVelocity, currentVelocity, dtVel);

    // 2. Calculate Angle PID Output
    float dtAng = angleTimer / 1000000.0;
    angleTimer = 0;
    float angOutput = calculateAnglePID(targetAngle, currentAngle, dtAng);

    // 3. Kinematic Mixing (The Blend)
    float powerLeft = velOutput + angOutput;
    float powerRight = velOutput - angOutput;

    const float MAX_POWER = 255.0;

    // 3. Find the maximum absolute power requested by either motor
    float maxRequested = max(abs(powerLeft), abs(powerRight));

    // 4. If we exceed limits, scale back ONLY the velocity component
    if (maxRequested > MAX_POWER) {
      // How much did we overflow?
      float overflow = maxRequested - MAX_POWER;

      // Reduce the velocity output directly by the overflow amount.
      // This perfectly shrinks the drive speed while keeping 'angOutput' untouched!
      if (velOutput > 0) {
        velOutput -= overflow;
      } else {
        velOutput += overflow;
      }

      // Recalculate safe, scaled powers
      powerLeft = velOutput + angOutput;
      powerRight = velOutput - angOutput;
    }

    //fitler and constrain
    powerLeft = constrain(powerLeft, -MAX_POWER, MAX_POWER);
    powerRight = constrain(powerRight, -MAX_POWER, MAX_POWER);


    // 4. Send the commands to the real hardware objects
    Serial.print(" L pow: ");
    Serial.print(powerLeft);
    Serial.print(" | R pow: ");
    Serial.print(powerRight);

    motorLeft.setPower(powerLeft);
    motorRight.setPower(powerRight);
  }



private:
  float calculateVelocityPID(float target, float current, float dt) {

    float error = target - current;

    // Serial.print(" Error: ");
    // Serial.print(error);

    velocityIntegral += error * dt;
    float derivative = (error - lastVelocityError) / dt;
    lastVelocityError = error;

    return (Kp_vel * error) + (Ki_vel * velocityIntegral) + (Kd_vel * derivative) + (Kf_vel * target);
  }

  float calculateAnglePID(float target, float current, float dt) {
    // CRITICAL: Handle angle wrapping (e.g., target 5 degrees, current 355 degrees)

    if (target != lastAngleTarget) {
      angleIntegral = 0;
    }

    lastAngleTarget = target;


    float error = target - current;
    if (error > 180) error -= 360;
    if (error < -180) error += 360;

    // Serial.print(" Error: ");
    // Serial.print(error);

    angleIntegral += error * dt;
    float derivative = (error - lastAngleError) / dt;
    lastAngleError = error;
    // Serial.print(" Intergral: ");
    // Serial.print(Ki_ang * angleIntegral);


    return (Kp_ang * error) + (Ki_ang * angleIntegral) + (Kd_ang * derivative) + (Kf_ang);
  }
};

float wheelVel(int vel0, int vel1) {
  return (vel0 + vel1) / 2.0;
}

float wheelAngle(int count0, int count1) {
  float angle = ((count0 - count1) / 1295.6) * 360.0;
  while (angle > 360) {
    angle = angle - 360;
  }
  while (angle < -360) {
    angle = angle + 360;
  }
  return angle;
}



Motor motorA0(A0_pw0, A0_pw1, A0_enc0, A0_enc1);
Motor motorA1(A1_pw0, A1_pw1, A1_enc0, A1_enc1);

Motor motorB0(B0_pw0, B0_pw1, B0_enc0, B0_enc1);
Motor motorB1(B1_pw0, B1_pw1, B1_enc0, B1_enc1);

wheelController wheelLeft(motorA0, motorA1);

wheelController wheelRight(motorB0, motorB1);



void motorA0ISR() {
  motorA0.updateEncoder();
}

void motorA1ISR() {
  motorA1.updateEncoder();
}

void motorB0ISR() {
  motorB0.updateEncoder();
}

void motorB1ISR() {
  motorB1.updateEncoder();
}

double stickAngle(double x, double y) {
  return atan2(y, x) * -57.2958;
}

double stickMagnitude(double x, double y) {
  if (abs(x) < 30) {
    x = 0;
  }
  if (abs(y) < 30) {
    y = 0;
  }
  return mapf(constrain(abs(sqrt(pow(x, 2) + pow(y, 2))), 0, 255), 0, 255, 0, 1);
}


float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float deadband(float input) {
  if (abs(input) < 5) return 0.0f;
  return input;
}


void music(int song1[][2], int song1Len, int song2[][2], int song2Len) {
  static int song1Index = 0;
  static int song2Index = 0;
  //reset the song from beginning
  if (PS4.Left()) {
    song1Index = 0;
    song2Index = 0;
    musicTimer1 = 0;
    musicTimer2 = 0;
  }

  if (song1Index < song1Len) {
    //play note
    ledcChangeFrequency(A0_pw0, song1[song1Index][0], 8);
    ledcChangeFrequency(A0_pw1, song1[song1Index][0], 8);
    Serial.print(" note number: ");
    Serial.print(song1Index);

    //setting power to play
    if (song1[song1Index][0] != 0) {
      motorA0.setPower(intensity);
    } else {
      motorA0.setPower(0);
    }

    //wait for note to finish
    if (musicTimer1 > song1[song1Index][1]) {
      musicTimer1 = 0;
      song1Index += 1;
    }
  }

  if (song2Index < song2Len) {
    //play note
    ledcChangeFrequency(A1_pw0, song2[song2Index][0], 8);
    ledcChangeFrequency(A1_pw1, song2[song2Index][0], 8);
    Serial.print(" note2 number: ");
    Serial.println(song2Index);

    if (song2[song2Index][0] != 0) {
      motorA1.setPower(intensity);
    } else {
      motorA1.setPower(0);
    }

    //wait for note to finish
    if (musicTimer2 > song2[song2Index][1]) {
      musicTimer2 = 0;
      song2Index += 1;
    }
  }
  //setting power

  //resetting back to normal for driving and stuff once both songs finish
  if (song1Index >= song1Len) {
    ledcChangeFrequency(A0_pw0, 20000, 8);
    ledcChangeFrequency(A0_pw1, 20000, 8);
  }
  if (song2Index >= song2Len) {
    ledcChangeFrequency(A1_pw0, 20000, 8);
    ledcChangeFrequency(A1_pw1, 20000, 8);
  }
}


void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32");
  Serial.println(SerialBT.getBtAddressString());
  PS4.begin("14:2b:2f:cd:7a:42");

  //PS4.begin("40:f5:20:45:22:8a");

  

  motorA0.begin();
  attachInterrupt(digitalPinToInterrupt(A0_enc0), motorA0ISR, CHANGE);

  motorA1.begin();
  attachInterrupt(digitalPinToInterrupt(A1_enc0), motorA1ISR, CHANGE);

  motorB0.begin();
  attachInterrupt(digitalPinToInterrupt(B0_enc0), motorB0ISR, CHANGE);

  motorB1.begin();
  attachInterrupt(digitalPinToInterrupt(B1_enc0), motorB1ISR, CHANGE);
}


void loop() {


  int rsticky = map(constrain(deadband(PS4.RStickY()), -120, 120), -120, 120, -255, 255);
  int rstickx = map(constrain(deadband(PS4.RStickX()), -120, 120), -120, 120, -255, 255);
  int lsticky = map(constrain(deadband(PS4.LStickY()), -120, 120), -120, 120, -255, 255);
  int lstickx = map(constrain(deadband(PS4.LStickX()), -120, 120), -120, 120, -255, 255);

  float l2 = mapf(PS4.L2Value(), 0.0, 255.0, 0.0, 1.0);


  int hertz = 20;
  int samplingRate = int(1000 / hertz);


  if (loopTimer > samplingRate && !PS4.L1()) {

    //LOW PASS FILTER
    float alpha = 0.35f;  // lower = smoother, more lag

    static float smoothed_ry = 0.0f;
    smoothed_ry = alpha * rsticky + (1.0f - alpha) * smoothed_ry;

    static float smoothed_rx = 0.0f;
    smoothed_rx = alpha * rstickx + (1.0f - alpha) * smoothed_rx;

    static float smoothed_l2 = 0.0f;
    smoothed_l2 = alpha * l2 + (1.0f - alpha) * smoothed_l2;

    static float smoothed_ly = 0.0f;
    smoothed_ly = alpha * lsticky + (1.0f - alpha) * smoothed_ly;

    static float smoothed_lx = 0.0f;
    smoothed_lx = alpha * lstickx + (1.0f - alpha) * smoothed_lx;

    static float sineMultiplier = 0.0f;
    sineMultiplier = sinf(radians( wheelAngle(motorA0.getEncoderCount(), motorA1.getEncoderCount())));

    static float velA = 0.0f;
    // velA = -maxVel * (stickMagnitude(smoothed_lx, smoothed_ly) + map(rstickx, -255, 255, -1, 1) );
    velA = -maxVel * (stickMagnitude(smoothed_lx, smoothed_ly) + map(rstickx, -255, 255, -1, 1) * sineMultiplier );

    static float velB = 0.0f;
    velB = -maxVel * (stickMagnitude(smoothed_lx, smoothed_ly) - (map(rstickx, -255, 255, -1, 1) * sineMultiplier ) );



    if (stickMagnitude(smoothed_rx, smoothed_ry) > 0.01) {
      angleTargetA = stickAngle(smoothed_rx, smoothed_ry);
    }

    if (stickMagnitude(smoothed_lx, smoothed_ly) > 0.01) {
      angleTargetB = stickAngle(smoothed_lx, smoothed_ly);
    }

    // Serial.print(l2 );

    Serial.print("wheelAngleA: ");
    Serial.print(wheelAngle(motorA0.getEncoderCount(), motorA1.getEncoderCount()));
    Serial.print("sine ");
    Serial.print( sineMultiplier );


    // Serial.print("  wheelAngleB: ");
    // Serial.print(wheelAngle(motorB0.getEncoderCount(), motorB1.getEncoderCount()));
    // Serial.print(" angletarget: ");
    // Serial.print(angleTarget);


    wheelLeft.update(velA, wheelVel(motorA0.getVelocity(), motorA1.getVelocity()),
                     angleTargetB, wheelAngle(motorA0.getEncoderCount(), motorA1.getEncoderCount()));

    wheelRight.update(velB, wheelVel(motorB0.getVelocity(), motorB1.getVelocity()),
                     -angleTargetB, wheelAngle(motorB0.getEncoderCount(), motorB1.getEncoderCount()));
    
    //tester

    // if(PS4.R1()){
    //   motorB1.setPower(255);
      
    // }else{
    //   motorB1.setPower(0);
      
    // }

    // Serial.print("Enc count: ");

    // Serial.print(motorB1.getEncoderCount());

    loopTimer = 0;
    Serial.println();
  }

  //tester
  


  if (PS4.L1()) {
    music(part2, noteCount, part1, noteCount);
  }

  //reset pwm
  if (PS4.Square()) {
    ledcChangeFrequency(A1_pw0, 20000, 8);
    ledcChangeFrequency(A1_pw1, 20000, 8);
    ledcChangeFrequency(A0_pw0, 20000, 8);
    ledcChangeFrequency(A0_pw1, 20000, 8);
  }
}