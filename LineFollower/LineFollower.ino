const int PWMA = 11;  // Pololu drive A
const int AIN2 = 10;
const int AIN1 = 9;
const int STDBY = 8;
const int BIN1 = 7;  // Pololu drive B
const int BIN2 = 6;
const int PWMB = 5;
float Kp = 1.0 * 255;
float Kd = 0.1 * 255;
float Ki = 0.025*255;
int speedL = 10;
int speedR = 10;
float prevE = 0;
float prevT = 0;
float alpha = .6;
float filt_e = 0;
float totalE = 0;
float slow = 0.8;

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
  int sensorRight = constrain(map(analogRead(A1), 550, 915, 0, 255), 1, 255);
  int sensorMid = constrain(map(analogRead(A2), 550, 915, 0, 255), 1, 255);
  int sensorLeft = constrain(map(analogRead(A3), 550, 915, 0, 255), 1, 255);
  float error = (1.0 * (sensorRight - sensorLeft))/(max(sensorRight, sensorLeft));
  filt_e = error * alpha + (1 - alpha) * filt_e;
  float dError = (filt_e - prevE) / (1.0*t1 - prevT);
  prevE = filt_e;
  totalE += filt_e;

  totalE = constrain(totalE, -100/255, 100/255);


  Serial.print(sensorRight);
  Serial.print(" ");
  Serial.print(sensorLeft);
  Serial.print(" ");
  Serial.print(filt_e);
  if (filt_e < 0) {
    speedL = 255 + (filt_e * Kp + Kd * dError + Ki * totalE*(t1 - prevT));
    speedR = 255;
  } else {
    speedL = 255;
    speedR = 255 - (filt_e * Kp + Kd * dError + Ki * totalE*(t1 - prevT));
  }
  speedL = constrain(speedL, 0, 255);
  speedR = constrain(speedR, 0, 255);
  Serial.print(" ");
  Serial.print(speedL);
  Serial.print(" ");
  Serial.println(speedR);
  prevT = t1;
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
  motorWrite(-speedL * 0.94, AIN1, AIN2, PWMA);
  motorWrite(speedR, BIN1, BIN2, PWMB);
}