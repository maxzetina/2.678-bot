const int PWMA = 11;  // Pololu drive A
const int AIN2 = 10;
const int AIN1 = 9;
const int STDBY = 8;
const int BIN1 = 7;  // Pololu drive B
const int BIN2 = 6;
const int PWMB = 5;

float Kp = 0.8*255;//1.0 * 255;
float Kd = 50.0*255;//500.0 * 255;
float Ki = 0*1.0*255;
int speedL, speedR;
float prevE = 0;
long prevT = 0;
float alpha = .85;
float filt_e = 0;
float totalE = 0;
float slow = 1.0;
long t1;
int sensorRight, sensorMid, sensorLeft;
float error, dError;
int numWhite = 0;

float deadZone = 0.0;



bool calibrate = false;
float timeout = 9.6;

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
  // Find current time
  t1 = millis();
  // Read the sensors
  sensorRight = constrain(map(analogRead(A1), 420-200, 730, 0, 255), 1, 255);
  sensorMid = constrain(map(analogRead(A2), 200, 730, 0, 255), 1, 255);
  sensorLeft = constrain(map(analogRead(A3), 420, 730, 0, 255), 1, 255);

  if(calibrate){
    Serial.print(sensorLeft);
    Serial.print(" ");
    Serial.print(sensorMid);
    Serial.print(" ");
    Serial.println(sensorRight);
    timeout = 1000;
  }

  // Serial.println(t1);

  if((t1 < 8.6 * 1000) && (t1 > 7.25 * 1000)){
    drive(255,243);
  }

  else{

  // See if it's over white, if not, run line follower
  if ((max(sensorRight,sensorLeft)>1) || (sensorMid>1) || t1 < timeout * 1000) {
    // Calculate error (s1-s2), use lowpass filter, find dError and totalE (constrained to avoid windup)
    error = (1.0 * (sensorRight - sensorLeft))/(max(sensorRight, sensorLeft));
    filt_e = error * alpha + (1 - alpha) * filt_e;
    dError = (filt_e - prevE) / (1.0*t1 - prevT);
    prevE = filt_e;
    totalE += filt_e;
    totalE = constrain(totalE, -100.0/255, 100.0/255);


    // Uses error to come up with speeds
    if (filt_e < -deadZone) {
      speedL = 255 + (filt_e * Kp + Kd * dError + Ki * totalE*(t1 - prevT));
      speedR = 255;
    } else if (filt_e > deadZone) {
      speedL = 255;
      speedR = 255 - (filt_e * Kp + Kd * dError + Ki * totalE*(t1 - prevT));
    }else{
      speedL=255;
      speedR=255;
    }
    
    speedL = constrain(speedL, 0, 255);
    speedR = constrain(speedR, 0, 255);
    prevT = t1;
    // Drives the wheels
    if(!calibrate){
      drive(speedL * slow, speedR * slow);  
    }
  }else{
    numWhite++;
    //If on white, do hard programed moves
    if (numWhite==1||numWhite==5) {
      //drive(255,255);
      if (numWhite == 1){
        Kp = 2.3 * 255;
        Kd = 210.0 * 255;
      }
      drive(255,0);
      while (!checkIfLine()){
        delay(1);
      }
    }else if (numWhite==2 || numWhite == 3){
      drive(255,255);
      Kp = 0.8*255;
      Kd = 50.0*255;
      while (!checkIfLine()){
        delay(1);
      }
    }else if (numWhite==4 || numWhite==6 || numWhite==7){
      if(numWhite==6){
        Kp=.6*255;
        Kd=50.0*255;
      }
      drive(0,255);
      while (!checkIfLine()){
        delay(1);
      }
    }else{
      drive(255,255);
      delay(2000);
    }
  }
  }
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

bool checkIfLine(){
  sensorRight = constrain(map(analogRead(A1), 580, 855, 0, 255), 1, 255);
  sensorMid = constrain(map(analogRead(A2), 365, 815, 0, 255), 1, 255);
  sensorLeft = constrain(map(analogRead(A3), 500, 855, 0, 255), 1, 255);
  return (max(sensorRight,sensorLeft)>100);
}

void drive(int speedL, int speedR) {
  motorWrite(-speedL * 0.88, AIN1, AIN2, PWMA);
  motorWrite(speedR, BIN1, BIN2, PWMB);
}