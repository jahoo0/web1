#include <Servo.h>

// DEFINE PIN NUMBER
#define MotorA_Low 22     // Motor Driver A in1 , Always Low  , BLUE line
#define MotorA_High 24     // Motor Driver A in2 , Always High  , GREEN line
#define PWM_A 8     // Motor Driver A pwm , PWM CONTROL PIN
#define pwma 23      // pwm inside motor, controled by Servo and value (255) blue line
#define MotorA_Dir 25     // direction line inside motor, (backward-HIGH,forward-LOW) orange line
#define encoderA 18    // Motor A encoder pin number

#define MotorB_Low 32     // Motor Driver B in3 , Always Low  , GREEN line
#define MotorB_High 30     // Motor Driver B in4 , Always High  , BLUE line
#define PWM_B 9     // Motor Driver B pwm , PWM CONTROL PIN
#define pwmb 31      // pwm inside motor, controled by Servo and value (255) blue line
#define MotorB_Dir 33     // direction line inside motor, (forward-HIGH,backward-LOW) orange line
#define encoderB 19    // Motor B encoder pin number

#define MotorC_Low 40     // Motor Driver C in1 , Always Low  , GREEN line
#define MotorC_High 38     // Motor Driver C in2 , Always High  , BLUE line
#define PWM_C 10     // Motor Driver C pwm , PWM CONTROL PIN
#define pwmc 39      // pwm inside motor, controled by Servo and value (255) blue line
#define MotorC_Dir 41     // direction line inside motor, (backward-HIGH,forward-LOW) orange line
#define encoderC 20    // Motor C encoder pin number

#define MotorD_Low 46     // Motor Driver D in1 , Always Low  , BLUE line
#define MotorD_High 48     // Motor Driver D in2 , Always High  , GREEN line
#define PWM_D 11     // Motor Driver C pwm , PWM CONTROL PIN
#define pwmd 47      // pwm inside motor, controled by Servo and value (255) blue line
#define MotorD_Dir 49     // direction line inside motor, (forward-HIGH,backward-LOW) orange line
#define encoderD 21    // Motor D encoder pin number


Servo MotorA_;    // Servo for pwm line inside motor (BLUE ONE)
Servo MotorB_;
Servo MotorC_;
Servo MotorD_;

int pwm_a = 50;    // CONTROL VALUE for each motor
int pwm_b = 50;
int pwm_c = 50;
int pwm_d = 50;

int encoderPos_A = 0;   // PULSE VALUE for encoder 
int flagA;              // To prevent repeated interrupt call for encoder
int encoderPos_B = 0;   // 
int flagB;              // 
int encoderPos_C = 0;   // 
int flagC;              // 
int encoderPos_D = 0;   // 
int flagD;              // 

double WIDTH = 0.155;
double LENGTH = 0.138;
double Rad = 0.05;

double measure_X;
double measure_Y;
double measure_W;

double targetVEL_X = 0.0;   // X-direction target velocity (m/s)
double targetVEL_Y = 0.0;   // Y-direction target velocity (m/s)
double targetVEL_W = 0.0;   // W-direction target angular velocity (rad/s) , positive means CCW direction

double targetRPM_A;     // Target velocity for each motor
double targetRPM_B;
double targetRPM_C;
double targetRPM_D;

double pulse_A;     
double rpm_A;
double pulse_B;     
double rpm_B;
double pulse_C;     
double rpm_C;
double pulse_D;     
double rpm_D;

double ratio = 32; // # of wings 
double threshold = 4.6875;

double error_A;
double error_prev_A = 0;
double de_A;
double cum_error_A = 0;
double error_B;
double error_prev_B = 0;
double de_B;
double cum_error_B = 0;
double error_C;
double error_prev_C = 0;
double de_C;
double cum_error_C = 0;
double error_D;
double error_prev_D = 0;
double de_D;
double cum_error_D = 0;

double kp = 0.35;
double ki = 0.0;
double kd = -0.02;

unsigned long ret_time = 0;     // Total run time
unsigned long time_prev = 1000;
unsigned long dt;

char val;

void setup() {
  pinMode(MotorA_High, OUTPUT);
  pinMode(MotorA_Low, OUTPUT);
  pinMode(MotorA_Dir, OUTPUT);
  pinMode(PWM_A, OUTPUT);
  pinMode(MotorB_High, OUTPUT);
  pinMode(MotorB_Low, OUTPUT);
  pinMode(MotorB_Dir, OUTPUT);
  pinMode(PWM_B, OUTPUT);
  pinMode(MotorC_High, OUTPUT);
  pinMode(MotorC_Low, OUTPUT);
  pinMode(MotorC_Dir, OUTPUT);
  pinMode(PWM_C, OUTPUT);
  pinMode(MotorD_High, OUTPUT);
  pinMode(MotorD_Low, OUTPUT);
  pinMode(MotorD_Dir, OUTPUT);
  pinMode(PWM_D, OUTPUT);

  pinMode(encoderA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderA), doEncoderA, CHANGE);
  pinMode(encoderB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderB), doEncoderB, CHANGE);
  pinMode(encoderC, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderC), doEncoderC, CHANGE);
  pinMode(encoderD, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderD), doEncoderD, CHANGE);

  MotorA_.attach(pwma);
  MotorB_.attach(pwmb);
  MotorC_.attach(pwmc);
  MotorD_.attach(pwmd);
  
//  MotorA_.write(0);

  Serial.begin(115200);
  Serial.println("Test driving ");

  delay(time_prev);
}

void loop() {

  while(Serial.available()){

    val = Serial.read();
    Serial.print("input data: ");
    Serial.println(val);
    if(val=='a'){
      targetVEL_X = 0.0;
    }
    else if(val=='s'){
      targetVEL_X = 0.1;
    }
    else if(val=='d'){
      targetVEL_X = 0.2;
    }
    else if(val=='f'){
      targetVEL_X = 0.3;
    }
    else if(val=='g'){
      targetVEL_X = 0.4;
    }
  }


  targetRPM_A = (targetVEL_X - targetVEL_Y - (WIDTH+LENGTH)*targetVEL_W) / Rad / (2*3.141592) * 60;
  targetRPM_B = (targetVEL_X + targetVEL_Y + (WIDTH+LENGTH)*targetVEL_W) / Rad / (2*3.141592) * 60;
  targetRPM_C = (targetVEL_X + targetVEL_Y - (WIDTH+LENGTH)*targetVEL_W) / Rad / (2*3.141592) * 60;
  targetRPM_D = (targetVEL_X - targetVEL_Y + (WIDTH+LENGTH)*targetVEL_W) / Rad / (2*3.141592) * 60;
  
  ret_time = (unsigned long)millis();
  if(false){
    stop();
  }     // 15초가 지나면 측정을 멈추도록 하는 코드
  else {
    
    drive();  
  
//    if (targetVEL_X < 0){
//      backward();
//    }
//    else if (targetVEL_X > 0) {
//      forward();
//    }
//    else {
//      stop();
//    }

    dt = (unsigned long)millis() - time_prev;
    if (dt>=200){
      time_prev = (unsigned long)millis();
      
      pulse_A = encoderPos_A;
      rpm_A = ( pulse_A / ratio )/(dt/1000.0) * 60;
      encoderPos_A = 0;
      
      pulse_B = encoderPos_B;
      rpm_B = ( pulse_B / ratio )/(dt/1000.0) * 60;
      encoderPos_B = 0;
      
      pulse_C = encoderPos_C;
      rpm_C = ( pulse_C / ratio )/(dt/1000.0) * 60;
      encoderPos_C = 0;
      
      pulse_D = encoderPos_D;
      rpm_D = ( pulse_D / ratio )/(dt/1000.0) * 60;
      encoderPos_D = 0;

      error_A = targetRPM_A - rpm_A;
      de_A = error_A - error_prev_A;
      error_prev_A = error_A;
      cum_error_A += error_A * dt/1000.0;
      error_B = targetRPM_B - rpm_B;
      de_B = error_B - error_prev_B;
      error_prev_B = error_B;
      cum_error_B += error_B * dt/1000.0;
      error_C = targetRPM_C - rpm_C;
      de_C = error_C - error_prev_C;
      error_prev_C = error_C;
      cum_error_C += error_C * dt/1000.0;
      error_D = targetRPM_D - rpm_D;
      de_D = error_D - error_prev_D;
      error_prev_D = error_D;
      cum_error_D += error_D * dt/1000.0;

      Serial.print("  rpmA = ");
      Serial.print(rpm_A,4);
      Serial.print("  targetRPM_A = ");
//      Serial.print(targetRPM_A,4);
//      Serial.print("  errorA = ");
//      Serial.print(error_A,2);
//      Serial.print("  deA = ");
//      Serial.print(de_A,2);
//      Serial.print("  cumerrorA = ");
//      Serial.print(cum_error_A,2);
//      Serial.print("  pwmA = ");
//      Serial.println(pwm_a);
      Serial.print("  rpmB = ");
      Serial.print(rpm_B,4);
      Serial.print("  errorB = ");
      Serial.print(error_B,2);
      Serial.print("  rpmC = ");
      Serial.print(rpm_C,4);
      Serial.print("  errorC = ");
      Serial.print(error_C,2);
      Serial.print("  rpmD = ");
      Serial.print(rpm_D,4);
      Serial.print("  errorD = ");
      Serial.println(error_D,2);
// A
      if ((targetRPM_A > 0) && (abs(error_A) > threshold)) { 
        if (pwm_a < 255){
          pwm_a += (kp*error_A) + (ki*cum_error_A) + (kd * de_A /(dt/1000.0));      // 
        }
        else if(pwm_a >= 255){
          pwm_a = 255;
        }
      }
      else if ((targetRPM_A < 0) && (abs(error_A) > threshold)) { 
        if (pwm_a < 255){
          pwm_a -= kp*error_A + ki*cum_error_A + kd*de_A/(dt/1000.0);      // 
        }
        else if(pwm_a >= 255){
          pwm_a = 255;
        }
      }
// B
      if ((targetRPM_B > 0) && (abs(error_B) > threshold)) { 
        if (pwm_b < 255){
          pwm_b += (kp*error_B) + (ki*cum_error_B) + (kd * de_B /(dt/1000.0));      // 
        }
        else if(pwm_b >= 255){
          pwm_b = 255;
        }
      }
      else if ((targetRPM_B < 0) && (abs(error_B) > threshold)) { 
        if (pwm_b < 255){
          pwm_b -= kp*error_B + ki*cum_error_B + kd*de_B/(dt/1000.0);      // 
        }
        else if(pwm_b >= 255){
          pwm_b = 255;
        }
      }
// C
      if ((targetRPM_C > 0) && (abs(error_C) > threshold)) { 
        if (pwm_c < 255){
          pwm_c += (kp*error_C) + (ki*cum_error_C) + (kd * de_C /(dt/1000.0));      // 
        }
        else if(pwm_c >= 255){
          pwm_c = 255;
        }
      }
      else if ((targetRPM_C < 0) && (abs(error_C) > threshold)) { 
        if (pwm_c < 255){
          pwm_c -= kp*error_C + ki*cum_error_C + kd*de_C/(dt/1000.0);      // 
        }
        else if(pwm_c >= 255){
          pwm_c = 255;
        }
      }
//D
      if ((targetRPM_D > 0) && (abs(error_D) > threshold)) { 
        if (pwm_d < 255){
          pwm_d += (kp*error_D) + (ki*cum_error_D) + (kd * de_D /(dt/1000.0));      // 
        }
        else if(pwm_d >= 255){
          pwm_d = 255;
        }
      }
      else if ((targetRPM_D < 0) && (abs(error_D) > threshold)) { 
        if (pwm_d < 255){
          pwm_d -= kp*error_D + ki*cum_error_D + kd*de_D/(dt/1000.0);      // 
        }
        else if(pwm_d >= 255){
          pwm_d = 255;
        }
      }

      
    }

    measure_X = (rpm_A + rpm_B + rpm_C + rpm_D) * (2*3.141592) * Rad / 4;
    measure_Y = (- rpm_A + rpm_B + rpm_C - rpm_D) * (2*3.141592) * Rad / 4;
    measure_W = (- rpm_A + rpm_B - rpm_C + rpm_D) * (2*3.141592) * Rad / 4 / (WIDTH + LENGTH);
  }
  
  flagA = 0;
  flagB = 0;
  flagC = 0;
  flagD = 0;
  
}

void drive() {
  digitalWrite(MotorA_High, HIGH);
  digitalWrite(MotorA_Low, LOW);
  if (targetRPM_A > 0) {
    digitalWrite(MotorA_Dir, LOW);   // for direction cw or ccw
    MotorA_.write(255);
    analogWrite(PWM_A,pwm_a);
  }
  else if (targetRPM_A < 0) {
    digitalWrite(MotorA_Dir, HIGH);   // for direction cw or ccw
    MotorA_.write(255);
    analogWrite(PWM_A,pwm_a);
  }
  else if (targetRPM_A == 0) {
    analogWrite(PWM_A,0);
  }

  digitalWrite(MotorB_High, HIGH);
  digitalWrite(MotorB_Low, LOW);
  if (targetRPM_B > 0) {
    digitalWrite(MotorB_Dir, HIGH);   // for direction cw or ccw
    MotorB_.write(255);
    analogWrite(PWM_B,pwm_b);
  }
  else if (targetRPM_B < 0) {
    digitalWrite(MotorB_Dir, LOW);   // for direction cw or ccw
    MotorB_.write(255);
    analogWrite(PWM_B,pwm_b);
  }
  else if (targetRPM_B == 0) {
    analogWrite(PWM_B,0);
  }

  digitalWrite(MotorC_High, HIGH);
  digitalWrite(MotorC_Low, LOW);
  if (targetRPM_C > 0) {
    digitalWrite(MotorC_Dir, LOW);   // for direction cw or ccw
    MotorC_.write(255);
    analogWrite(PWM_C,pwm_c);
  }
  else if (targetRPM_C < 0) {
    digitalWrite(MotorC_Dir, HIGH);   // for direction cw or ccw
    MotorC_.write(255);
    analogWrite(PWM_C,pwm_c);
  }
  else if (targetRPM_C == 0) {
    analogWrite(PWM_C,0);
  }

  digitalWrite(MotorD_High, HIGH);
  digitalWrite(MotorD_Low, LOW);
  if (targetRPM_D > 0) {
    digitalWrite(MotorD_Dir, HIGH);   // for direction cw or ccw
    MotorD_.write(255);
    analogWrite(PWM_D,pwm_d);
  }
  else if (targetRPM_D < 0) {
    digitalWrite(MotorD_Dir, LOW);   // for direction cw or ccw
    MotorD_.write(255);
    analogWrite(PWM_D,pwm_d);
  }
  else if (targetRPM_D == 0) {
    analogWrite(PWM_D,0);
  }
}

void stop(){
  analogWrite(PWM_A, 0);
  analogWrite(PWM_B, 0);
  analogWrite(PWM_C, 0);
  analogWrite(PWM_D, 0);
}

void doEncoderA(){
  if(flagA==0){
  encoderPos_A += (targetRPM_A>0)?1:-1;
  flagA=1;
  }
}
void doEncoderB(){
  if(flagB==0){
  encoderPos_B += (targetRPM_B>0)?1:-1;
  flagB=1;
  }
}
void doEncoderC(){
  if(flagC==0){
  encoderPos_C += (targetRPM_C>0)?1:-1;
  flagC=1;
  }
}
void doEncoderD(){
  if(flagD==0){
  encoderPos_D += (targetRPM_D>0)?1:-1;
  flagD=1;
  }
}

//////////////////////////////////////
