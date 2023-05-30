#include <ECE3.h>

const int left_nslp_pin=31; 
const int right_nslp_pin=11; 
const int left_dir_pin=29;
const int right_dir_pin=30;
const int left_pwm_pin=40;
const int right_pwm_pin=39;

short unsigned int rawSensor[8];
int sensor[8];
int ambient[8] = {641, 688, 595, 664, 595, 595, 641, 734}; 
float weighting[8] = {0.5467468562, 0.5518763797, 0.6108735492, 0.5844535359, 0.6566850538, 0.5854800937, 0.5662835249, 0.5662514156};
int scheme[8] = {-8, -5, -3, -1, 1, 3, 5, 8};
float error = 0;
float oldError = error;

float baseSpeed[9] = {65, 80, 170, 30, 45, 60, 80, 80, 65};
float k[9][2] = {
  {0.005, 0.0020}, // start
  {0.012, 0.0035}, // turn
  {0.01, 0.0025}, // straight
  {0.007, 0.0045}, // weird bit
  {0.012, 0.0035}, // ending
  {0.005, 0.0020},
  {0.008, 0.0020}, // straight
  {0.012, 0.0035}, // turn
  {0.005, 0.002}}; // start
int section = 0;
int triggers[8] = {300, 1900, 3300, 3500, 4500, 6350, 8000, 9000};
// trigger         start turn straightweirdend  back  turn   end

  
float Kp = k[0][0];
float Kd = k[0][1];


float leftSpeed = 0;
float rightSpeed = 0;

int turnaround = 0;

void setup()
{
  ECE3_Init();

  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);
  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);

  digitalWrite(left_nslp_pin,HIGH);
  digitalWrite(right_nslp_pin,HIGH);

  digitalWrite(left_dir_pin,LOW);  // Set car direction to forward
  digitalWrite(right_dir_pin,LOW);
  
  //Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  delay(2000);
  resetEncoderCount_left();
  resetEncoderCount_right();

  ECE3_read_IR(rawSensor);
  setError();
  oldError = error;
  ChangeWheelSpeeds(baseSpeed[section], baseSpeed[section]);
}


void loop()
{
  ECE3_read_IR(rawSensor);
  setError();

  if (avgEncoder() > triggers[section]) {
    section++;
    Kp = k[section][0];
    Kd = k[section][1];
    ChangeWheelSpeeds(baseSpeed[section], baseSpeed[section]);

    if (section == 3) {
      scheme[6] = scheme[7] = 0;
    }
    else if (section == 4) {
      scheme[6] = 5;
      scheme[7] = 8;
    }
  }


  if (avgEncoder() > 9000 && rawSensor[2] > 1000 && rawSensor[4] > 1000 && rawSensor[3] > 1000 && rawSensor[5] > 1000) {
    ECE3_read_IR(rawSensor);
    setError();
    if (rawSensor[2] > 1000 && rawSensor[4] > 1000 && rawSensor[3] > 1000 && rawSensor[5] > 1000) {
      ChangeWheelSpeeds(0, 0);
      exit(0);
    }
  }

  else if (turnaround == 0 && rawSensor[2] > 1000 && rawSensor[4] > 1000 && rawSensor[3] > 1000 && rawSensor[5] > 1000) {
    ECE3_read_IR(rawSensor);
    setError();
    if (rawSensor[2] > 1000 && rawSensor[4] > 1000 && rawSensor[3] > 1000 && rawSensor[5] > 1000) {
      digitalWrite(left_dir_pin,HIGH);
      ChangeWheelSpeeds(150, 150);
      delay(350);
      digitalWrite(left_dir_pin,LOW);
      ChangeWheelSpeeds(baseSpeed[section], baseSpeed[section]);
      turnaround++;
    }
  }


  analogWrite(left_pwm_pin, baseSpeed[section] - Kp * error - Kd * (error - oldError));
    analogWrite(right_pwm_pin, baseSpeed[section] + Kp * error + Kd * (error - oldError));
  
  oldError = error;
}

void setError() {
  error = 0;
  for (int i = 0; i < 8; i++) {
    sensor[i] = rawSensor[i];
    sensor[i] -= ambient[i];
    sensor[i] *= weighting[i];
    error += sensor[i] * scheme[i];
  }
}

void ChangeWheelSpeeds(int finalLeftSpd,int finalRightSpd) {
  
  int diffLeft = finalLeftSpd-leftSpeed;
  int diffRight = finalRightSpd-rightSpeed;
  int stepIncrement = 20;
  int numStepsLeft = abs(diffLeft)/stepIncrement;
  int numStepsRight = abs(diffRight)/stepIncrement;
  int numSteps = max(numStepsLeft,numStepsRight);
  int pwmLeftVal = leftSpeed; // initialize left wheel speed
  int pwmRightVal = rightSpeed; // initialize right wheel speed
  int deltaLeft = (diffLeft)/numSteps; // left in(de)crement
  int deltaRight = (diffRight)/numSteps; // right in(de)crement
  for (int k = 0; k < numSteps; k++) {
    pwmLeftVal = pwmLeftVal + deltaLeft;
    pwmRightVal = pwmRightVal + deltaRight;
    analogWrite(left_pwm_pin,pwmLeftVal);
    analogWrite(right_pwm_pin,pwmRightVal);
    delay(30);
  }
  analogWrite(left_pwm_pin,finalLeftSpd);
  analogWrite(right_pwm_pin,finalRightSpd);
  rightSpeed = finalRightSpd;
  leftSpeed = finalLeftSpd;
}

int avgEncoder() {
  return ((getEncoderCount_left() + getEncoderCount_right()) / 2);
}
