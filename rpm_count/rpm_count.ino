// code start
#define PIN_A2 9
#define PIN_B2 8
#define PWM 10
#define encoderA 2
#define encoderB 3

int encoderPos = 0;
double pulse;
double rpm;
double motorDeg;
double ratio = 360.0/29/12;

unsigned long ret_time = 0;
unsigned long time_prev = 0;
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
  Serial.println("Motor Test Start");
}

void loop() {

  ret_time = (unsigned long)millis();
  if(ret_time >= 30000){
    stop();
  }
  else{

  forward();
  
  dt = (unsigned long)millis() - time_prev;
  if (dt >= 1000){
    time_prev = (unsigned long)millis();
    pulse = encoderPos;
    rpm = (pulse/(29.0*12))*60;
    encoderPos = 0;

    motorDeg = pulse * ratio;
    Serial.print("Deg = ");
    Serial.print(motorDeg);
    Serial.print("    pulse = ");
    Serial.print(pulse);
    Serial.print("    dt = ");
    Serial.print(dt);
    Serial.print("    RPM = ");
    Serial.println(rpm,4);
  }
  }
}

void forward(){
  digitalWrite(PIN_A2, HIGH);
  digitalWrite(PIN_B2, LOW);
  analogWrite(PWM, 100);
}

void backward(){
  digitalWrite(PIN_A2, LOW);
  digitalWrite(PIN_B2, HIGH);
  analogWrite(PWM, 100);
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
