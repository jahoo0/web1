 // code start
#define PIN_A2 9
#define PIN_B2 8
#define PWM 10
#define encoderA 2
#define encoderB 3

int encoderPos = 0;
int pwm_value = 100;
double pulse;
double rpm;
double motorDeg;
double ratio = 360.0/29/12;
double targetRPM = 150.0;
double error;
double threshold = 0.5;
double error_prev = 0;
double de;

unsigned long ret_time = 0;
unsigned long time_prev = 1000;
unsigned long dt;

char val;

void setup() {
  pinMode(PIN_A2, OUTPUT);
  pinMode(PIN_B2, OUTPUT);

  pinMode(encoderA, INPUT_PULLUP);
  attachInterrupt(2, doEncoderA, CHANGE);
 
  pinMode(encoderB, INPUT_PULLUP);
  attachInterrupt(3, doEncoderB, CHANGE);

  Serial.begin(115200);
  delay(1000);
  Serial.println("Motor Test Start");
}

void loop() {
      
  ret_time = (unsigned long)millis();
  if(ret_time >= 15000){
    stop();
  }
  else{

    if (targetRPM < 0){
      backward();
    }
    else if (targetRPM > 0) {
      forward();
    }
    else {
      stop();
    }
  
  dt = (unsigned long)millis() - time_prev;
  if (dt >= 200){
    time_prev = (unsigned long)millis();
    pulse = encoderPos;
    rpm = ( pulse / (29.0 * 12) )/(dt/1000.0) * 60;
    encoderPos = 0;

    error = targetRPM - rpm;
    de = error - error_prev;
    error_prev = error;

    motorDeg = pulse * ratio;
    Serial.print("RPM = ");
    Serial.print(rpm,4);
    Serial.print("    pwm = ");
    Serial.print(pwm_value);
    Serial.print("    error = ");
    Serial.print(error);
    Serial.print("    dt = ");
    Serial.print(dt);
    Serial.print("    de = ");
    Serial.println(de);

    if(encoderPos == 0  && targetRPM > 0){
      if (abs(error) <= threshold){
        Serial.println("excellent");
      }
      else if (error > threshold){
        pwm_value += error/1.0 - abs(de)/2.0;
      }
      else if (error < -threshold){
        pwm_value -= abs(error)/1.0 + abs(de)/2.0;
      }
    }

    if(encoderPos == 0  && targetRPM < 0){
      if (abs(error) <= threshold){
        Serial.println("excellent");
      }
      else if (error > threshold){
        pwm_value -= error/1.0 - abs(de)/2.0;
      }
      else if (error < -threshold){
        pwm_value += abs(error)/1.0 + abs(de)/2.0;
      }
    }
  }
  }
}

void forward(){
  digitalWrite(PIN_A2, HIGH);
  digitalWrite(PIN_B2, LOW);
  analogWrite(PWM, pwm_value);
}

void backward(){
  digitalWrite(PIN_A2, LOW);
  digitalWrite(PIN_B2, HIGH);
  analogWrite(PWM, pwm_value);
}

void stop(){
  digitalWrite(PIN_A2, LOW);
  digitalWrite(PIN_B2, LOW);
  analogWrite(PWM, 0);
}

void doEncoderA(){ // 빨녹일 때
//  if(digitalRead(encoderPinA)==digitalRead(encoderPinB)) // 같으면
//    encoderPos++; // 정회전
//  else // 다르면
//    encoderPos--; // 역회전
  encoderPos += (digitalRead(encoderA)==digitalRead(encoderB))?1:-1;
}

void doEncoderB(){ // 보파일 때
//  if(digitalRead(encoderPinA)==digitalRead(encoderPinB)) // 같으면
//    encoderPos--; // 역회전
//  else // 다르면
//    encoderPos++; // 정회전
  encoderPos += (digitalRead(encoderA)==digitalRead(encoderB))?-1:1;
}
// code end
