 // code start
#define MotorA_CW 22
#define MotorA_CCW 24
#define MotorA_PWM 26
#define MotorA_ENC 2

#define MotorB_CW 30
#define MotorB_CCW 32
#define MotorB_PWM 34
#define MotorB_ENC 3

#define MotorC_CW 38
#define MotorC_CCW 40
#define MotorC_PWM 42
#define MotorC_ENC 4

#define MotorD_CW 46
#define MotorD_CCW 48
#define MotorD_PWM 50
#define MotorD_ENC 5

int encoderPos_A = 0;
int encoderPos_B = 0;
int encoderPos_C = 0;
int encoderPos_D = 0;
int pwm_A = 100;
int pwm_B = 100;
int pwm_C = 100;
int pwm_D = 100;
int wings = 16;

double targetRPM_A = 150.0;
double targetRPM_B = 150.0;
double targetRPM_C = 150.0;
double targetRPM_D = 150.0;
double RPM_A;
double RPM_B;
double RPM_C;
double RPM_D;
double error_A;
double error_B;
double error_C;
double error_D;
double error_prev_A = 0;
double error_prev_B = 0;
double error_prev_C = 0;
double error_prev_D = 0;
double de_A;
double de_B;
double de_C;
double de_D;
double threshold = 1;

unsigned long time_prev = 1000;
unsigned long dt;

void setup() {
  pinMode(MotorA_CW, OUTPUT);
  pinMode(MotorA_CCW, OUTPUT);
  pinMode(MotorB_CW, OUTPUT);
  pinMode(MotorB_CCW, OUTPUT);
  pinMode(MotorC_CW, OUTPUT);
  pinMode(MotorC_CCW, OUTPUT);
  pinMode(MotorD_CW, OUTPUT);
  pinMode(MotorD_CCW, OUTPUT);

  pinMode(MotorA_ENC, INPUT_PULLUP);
  attachInterrupt(2, doEncoderA, CHANGE);
  pinMode(MotorB_ENC, INPUT_PULLUP);
  attachInterrupt(3, doEncoderB, CHANGE);
  pinMode(MotorC_ENC, INPUT_PULLUP);
  attachInterrupt(4, doEncoderC, CHANGE);
  pinMode(MotorD_ENC, INPUT_PULLUP);
  attachInterrupt(5, doEncoderD, CHANGE);

  Serial.begin(115200);
  delay(1000);
  Serial.println("Motor Test Start");
}

void loop() {
  dt = (unsigned long)millis() - time_prev;
  
  if (targetRPM_A < 0){
      backward_A();
    }
  else if (targetRPM_A > 0) {
      forward_A();
    }
  else {
      Stop_A();
    }

  if (targetRPM_B < 0){
      backward_B();
    }
  else if (targetRPM_B > 0) {
      forward_B();
    }
  else {
      Stop_B();
    }

  if (targetRPM_C < 0){
      backward_C();
    }
  else if (targetRPM_C > 0) {
      forward_C();
    }
  else {
      Stop_C();
    }

  if (targetRPM_D < 0){
      backward_D();
    }
  else if (targetRPM_D > 0) {
      forward_D();
    }
  else {
      Stop_D();
    }
    
  if (dt>=200){
    time_prev = (unsigned long)millis();
    RPM_A = ( encoderPos_A / ( wings ) ) / (dt/1000.0) * 60;
    RPM_B = ( encoderPos_B / ( wings ) ) / (dt/1000.0) * 60;
    RPM_C = ( encoderPos_C / ( wings ) ) / (dt/1000.0) * 60;
    RPM_D = ( encoderPos_D / ( wings ) ) / (dt/1000.0) * 60;
    encoderPos_A = 0;
    encoderPos_B = 0;
    encoderPos_C = 0;
    encoderPos_D = 0;

    error_A = targetRPM_A - RPM_A;
    error_B = targetRPM_B - RPM_B;
    error_C = targetRPM_C - RPM_C;
    error_D = targetRPM_D - RPM_D;
    de_A = error_A - error_prev_A;
    de_B = error_B - error_prev_B;
    de_C = error_C - error_prev_C;
    de_D = error_D - error_prev_D;
    error_prev_A = error_A;
    error_prev_B = error_B;
    error_prev_C = error_C;
    error_prev_D = error_D;

    if (targetRPM_A > 0 && error_A > threshold){
      PWM_A += error_A/1.0 - abs(de_A)/2.0;
    }
    else if (targetRPM_A > 0 && error_A < -threshold){
      PWM_A -= error_A/1.0 + abs(de_A)/2.0;
    }
    else if (targetRPM_A < 0 && error_A > threshold){
      PWM_A -= error_A/1.0 - abs(de_A)/2.0;
    }
    else if (targetRPM_A < 0 && error_A < -threshold){
      PWM_A += error_A/1.0 + abs(de_A)/2.0;
    }

    if (targetRPM_B > 0 && error_B > threshold){
      PWM_B += error_B/1.0 - abs(de_B)/2.0;
    }
    else if (targetRPM_B > 0 && error_B < -threshold){
      PWM_B -= error_B/1.0 + abs(de_B)/2.0;
    }
    else if (targetRPM_B < 0 && error_B > threshold){
      PWM_B -= error_B/1.0 - abs(de_B)/2.0;
    }
    else if (targetRPM_B < 0 && error_B < -threshold){
      PWM_B += error_B/1.0 + abs(de_B)/2.0;
    }

    if (targetRPM_C > 0 && error_C > threshold){
      PWM_C += error_C/1.0 - abs(de_C)/2.0;
    }
    else if (targetRPM_C > 0 && error_C < -threshold){
      PWM_C -= error_C/1.0 + abs(de_C)/2.0;
    }
    else if (targetRPM_C < 0 && error_C > threshold){
      PWM_C -= error_C/1.0 - abs(de_C)/2.0;
    }
    else if (targetRPM_C < 0 && error_C < -threshold){
      PWM_C += error_C/1.0 + abs(de_C)/2.0;
    }

    if (targetRPM_D > 0 && error_D > threshold){
      PWM_D += error_D/1.0 - abs(de_D)/2.0;
    }
    else if (targetRPM_D > 0 && error_D < -threshold){
      PWM_D -= error_D/1.0 + abs(de_D)/2.0;
    }
    else if (targetRPM_D < 0 && error_D > threshold){
      PWM_D -= error_D/1.0 - abs(de_D)/2.0;
    }
    else if (targetRPM_D < 0 && error_D < -threshold){
      PWM_D += error_D/1.0 + abs(de_D)/2.0;
    }
    
    Serial.print("RPM = ");
    Serial.print(RPM_A,4);
    Serial.print("  ");
    Serial.print(RPM_B,4);
    Serial.print("  ");
    Serial.print(RPM_C,4);
    Serial.print("  ");
    Serial.println(RPM_D,4);
  }
}

void forward_A(){
  digitalWrite(MotorA_CW, HIGH);
  digitalWrite(MotorA_CCW, LOW);
  analogWrite(MotorA_PWM, PWM_A);
}
void forward_B(){
  digitalWrite(MotorB_CW, HIGH);
  digitalWrite(MotorB_CCW, LOW);
  analogWrite(MotorB_PWM, PWM_B);
}
void forward_C(){
  digitalWrite(MotorC_CW, HIGH);
  digitalWrite(MotorC_CCW, LOW);
  analogWrite(MotorC_PWM, PWM_C);
}
void forward_D(){
  digitalWrite(MotorD_CW, HIGH);
  digitalWrite(MotorD_CCW, LOW);
  analogWrite(MotorD_PWM, PWM_D);
}

void backward_A(){
  digitalWrite(MotorA_CW, LOW);
  digitalWrite(MotorA_CCW, HIGH);
  analogWrite(MotorA_PWM, PWM_A);
}
void backward_B(){
  digitalWrite(MotorB_CW, LOW);
  digitalWrite(MotorB_CCW, HIGH);
  analogWrite(MotorB_PWM, PWM_B);
}
void backward_C(){
  digitalWrite(MotorC_CW, LOW);
  digitalWrite(MotorC_CCW, HIGH);
  analogWrite(MotorC_PWM, PWM_C);
}
void backward_D(){
  digitalWrite(MotorD_CW, LOW);
  digitalWrite(MotorD_CCW, HIGH);
  analogWrite(MotorD_PWM, PWM_D);
}

void Stop_A(){
  digitalWrite(MotorA_CW, LOW);
  digitalWrite(MotorA_CCW, LOW);
  analogWrite(MotorA_PWM, 0);
}
void Stop_B(){
  digitalWrite(MotorB_CW, LOW);
  digitalWrite(MotorB_CCW, LOW);
  analogWrite(MotorB_PWM, 0);
}
void Stop_C(){
  digitalWrite(MotorC_CW, LOW);
  digitalWrite(MotorC_CCW, LOW);
  analogWrite(MotorC_PWM, 0);
}
void Stop_D(){
  digitalWrite(MotorD_CW, LOW);
  digitalWrite(MotorD_CCW, LOW);
  analogWrite(MotorD_PWM, 0);
}

void doEncoderA(){ 
  encoderPos_A += (targetRPM_A>=0)?1:-1;
}
void doEncoderB(){ 
  encoderPos_B += (targetRPM_B>=0)?1:-1;
}
void doEncoderC(){ 
  encoderPos_C += (targetRPM_C>=0)?1:-1;
}
void doEncoderD(){ 
  encoderPos_D += (targetRPM_D>=0)?1:-1;
}
