const int PWMA = 11;  // Pololu drive A
const int AIN2 = 10;
const int AIN1 = 9;
const int STDBY = 8;
const int BIN1 = 7;  // Pololu drive B
const int BIN2 = 6;
const int PWMB = 5;
float Kp = 1.0;
float Kd = .3;
int speedL = 10;
int speedR = 10;
float prevE = 0;
float prevT = 0;
float alpha = .6;
float filt_e = 0;
float slow = .4;

void setup() {
  Serial.begin(115200);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STDBY, OUTPUT);
  digitalWrite(STDBY, HIGH);

  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
}

void loop() {
  float t1 = millis();
  int sensorRight = map(analogRead(A1), 100, 1000, 0, 255);
  int sensorMid = map(analogRead(A2), 100, 1000, 0, 255);
  int sensorLeft = map(analogRead(A3), 100, 1000, 0, 255);
  float error = 1.0 * (sensorRight - sensorLeft);
  float dError = (error - prevE) / (t1 - prevT);
  filt_e = error * alpha + (1 - alpha) * filt_e;
  prevE = filt_e;
  prevT = t1;

  // Serial.print(sensorRight);
  // Serial.print(sensorMid);
  Serial.println(error);
  if (error < 0) {
    speedL = 255 - (error * Kp + Kd * dError);
    speedR = 255;
  } else {
    speedL = 255;
    speedR = 255 + (error * Kp + Kd * dError);
  }
  speedL = constrain(speedL, 0, 255);
  speedR = constrain(speedR, 0, 255);
  drive(speedL * slow, speedR * slow);  //drive forwards
}

void motorWrite(int spd, int pin_IN1, int pin_IN2, int pin_PWM) {
  if (spd < 0) {
    digitalWrite(pin_IN1, HIGH);  // go one way
    digitalWrite(pin_IN2, LOW);
  } else {
    digitalWrite(pin_IN1, LOW);  // go the other way
    digitalWrite(pin_IN2, HIGH);
  }
  analogWrite(pin_PWM, abs(spd));
}

void drive(int speedL, int speedR) {
  motorWrite(-speedL, AIN1, AIN2, PWMA);
  motorWrite(speedR * 0.935, BIN1, BIN2, PWMB);
}