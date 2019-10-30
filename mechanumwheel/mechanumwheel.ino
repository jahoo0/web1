// 네 바퀴 용

#include <Servo.h>

// pin 번호 맞게 수정하세요잉
#define MotorA_Low 22     // 모터 드라이버A in1 번호, 항상 Low  , BLUE
#define MotorA_High 24     // 모터 드라이버A in2 번호, 항상 High로 해줘야함  , GREEN
#define PWM_A 8     // 모터 드라이버 pwm 번호 , 이 핀 번호로 pwm값 제어 들어갈 것
#define pwma 23      // 모터 내장형 pwm, Servo로 컨트롤하며 항상 최대값 (255) 파랑색 선
#define MotorA_Dir 25     // 모터 내장형 direction line (CW-HIGH,CCW-LOW) 주황색 선
#define encoderA 18    // 모터A 용 광학용 엔코더 pin번호

#define MotorB_Low 32     // 모터 드라이버 in4 번호, 항상 Low  , BLUE
#define MotorB_High 30     // 모터 드라이버 in3 번호, 항상 High로 해줘야함  , GREEN
#define PWM_B 9     // 모터 드라이버 pwm 번호 , 이 핀 번호로 pwm값 제어 들어갈 것
#define pwmb 31      // 모터 내장형 pwm, Servo로 컨트롤하며 항상 최대값 (255) 파랑색 선
#define MotorB_Dir 33     // 모터 내장형 direction line (CW-HIGH,CCW-LOW) 주황색 선
#define encoderB 19    // 모터B 용 광학용 엔코더 pin번호

#define MotorC_Low 40     // 모터 드라이버 in4 번호, 항상 Low  , BLUE
#define MotorC_High 38     // 모터 드라이버 in3 번호, 항상 High로 해줘야함  , GREEN
#define PWM_C 10     // 모터 드라이버 pwm 번호 , 이 핀 번호로 pwm값 제어 들어갈 것
#define pwmc 39      // 모터 내장형 pwm, Servo로 컨트롤하며 항상 최대값 (255) 파랑색 선
#define MotorC_Dir 41     // 모터 내장형 direction line (CW-HIGH,CCW-LOW) 주황색 선
#define encoderC 20    // 모터C 용 광학용 엔코더 pin번호

#define MotorD_Low 46     // 모터 드라이버D in1 번호, 항상 Low  , BLUE
#define MotorD_High 48     // 모터 드라이버D in2 번호, 항상 High로 해줘야함  , GREEN
#define PWM_D 11     // 모터 드라이버D pwm 번호 , 이 핀 번호로 pwm값 제어 들어갈 것
#define pwmd 47      // 모터D 내장형 pwm, Servo로 컨트롤하며 항상 최대값 (255) 파랑색 선
#define MotorD_Dir 49     // 모터 내장형D direction line (CW-HIGH,CCW-LOW) 주황색 선
#define encoderD 21    // 모터D 용 광학용 엔코더 pin번호

Servo MotorA_;
Servo MotorB_;
Servo MotorC_;
Servo MotorD_;

int pwm_a = 50;
int pwm_b = 50;
int pwm_c = 50;
int pwm_d = 50;

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

  MotorA_.attach(pwma);
  MotorB_.attach(pwmb);
  MotorC_.attach(pwmc);
  MotorD_.attach(pwmd);

//  MotorA_.write(0);

  Serial.begin(115200);
  Serial.print("Test driving ");

  delay(500);
}

void loop() {
//  while(!Serial.available()); // 시리얼 입력 받을 때 까지 기다리기
//  while(Serial.available()){
//    val = Serial.read();
//    Serial.print("input data: ");
//    Serial.println(val);
//    if(val=='f'){
//      forward();
//      Serial.println("forward");
//    } else if(val=='b'){
//      backward();
//      Serial.println("backward"); 
//    } else if(val=='l'){
//      left();
//      Serial.println("left"); 
//    } else if(val=='r'){
//      right();
//      Serial.println("right"); 
//    } else if(val=='t'){
//      turn_cw();
//      Serial.println("clockwise"); 
//    } else if(val=='y'){
//      turn_ccw();
//      Serial.println("counter-clockwise"); 
//    } else if(val=='s'){
//      stop();
//      Serial.println("stop");
//    }
//  }

  delay(2000);
  forward();
  delay(1300);
  backward();
  delay(1300);
  
  right();
  delay(1300);
  
  left();
  delay(1300);
  
  turn_cw();
  delay(1300);
  
  turn_ccw();
  delay(1300);

  stop();
  delay(200000);

  
}

void forward(){
  digitalWrite(MotorA_High, HIGH);
  digitalWrite(MotorA_Low, LOW);
  digitalWrite(MotorA_Dir, LOW);   // for direction cw or ccw
  MotorA_.write(255);
  analogWrite(PWM_A,pwm_a);

  digitalWrite(MotorB_High, HIGH);
  digitalWrite(MotorB_Low, LOW);
  digitalWrite(MotorB_Dir, HIGH);
  MotorB_.write(255);
  analogWrite(PWM_B,pwm_b);

  digitalWrite(MotorC_High, HIGH);
  digitalWrite(MotorC_Low, LOW);
  digitalWrite(MotorC_Dir, LOW);
  MotorC_.write(255);
  analogWrite(PWM_C,pwm_c);

  digitalWrite(MotorD_High, HIGH);
  digitalWrite(MotorD_Low, LOW);
  digitalWrite(MotorD_Dir, HIGH);
  MotorD_.write(255);
  analogWrite(PWM_D,pwm_d);
}

void backward(){
  digitalWrite(MotorA_High, HIGH);
  digitalWrite(MotorA_Low, LOW);
  digitalWrite(MotorA_Dir, HIGH);   // for direction cw or ccw
  MotorA_.write(255);
  analogWrite(PWM_A,pwm_a);

  digitalWrite(MotorB_High, HIGH);
  digitalWrite(MotorB_Low, LOW);
  digitalWrite(MotorB_Dir, LOW);
  MotorB_.write(255);
  analogWrite(PWM_B,pwm_b);

  digitalWrite(MotorC_High, HIGH);
  digitalWrite(MotorC_Low, LOW);
  digitalWrite(MotorC_Dir, HIGH);
  MotorC_.write(255);
  analogWrite(PWM_C,pwm_c);

  digitalWrite(MotorD_High, HIGH);
  digitalWrite(MotorD_Low, LOW);
  digitalWrite(MotorD_Dir, LOW);
  MotorD_.write(255);
  analogWrite(PWM_D,pwm_d);
}

void left(){
  digitalWrite(MotorA_High, HIGH);
  digitalWrite(MotorA_Low, LOW);
  digitalWrite(MotorA_Dir, HIGH);   // for direction cw or ccw
  MotorA_.write(255);
  analogWrite(PWM_A,pwm_a);

  digitalWrite(MotorB_High, HIGH);
  digitalWrite(MotorB_Low, LOW);
  digitalWrite(MotorB_Dir, HIGH);
  MotorB_.write(255);
  analogWrite(PWM_B,pwm_b);

  digitalWrite(MotorC_High, HIGH);
  digitalWrite(MotorC_Low, LOW);
  digitalWrite(MotorC_Dir, LOW);
  MotorC_.write(255);
  analogWrite(PWM_C,pwm_c);

  digitalWrite(MotorD_High, HIGH);
  digitalWrite(MotorD_Low, LOW);
  digitalWrite(MotorD_Dir, LOW);
  MotorD_.write(255);
  analogWrite(PWM_D,pwm_d);
}

void right(){
  digitalWrite(MotorA_High, HIGH);
  digitalWrite(MotorA_Low, LOW);
  digitalWrite(MotorA_Dir, LOW);   // for direction cw or ccw
  MotorA_.write(255);
  analogWrite(PWM_A,pwm_a);

  digitalWrite(MotorB_High, HIGH);
  digitalWrite(MotorB_Low, LOW);
  digitalWrite(MotorB_Dir, LOW);
  MotorB_.write(255);
  analogWrite(PWM_B,pwm_b);

  digitalWrite(MotorC_High, HIGH);
  digitalWrite(MotorC_Low, LOW);
  digitalWrite(MotorC_Dir, HIGH);
  MotorC_.write(255);
  analogWrite(PWM_C,pwm_c);

  digitalWrite(MotorD_High, HIGH);
  digitalWrite(MotorD_Low, LOW);
  digitalWrite(MotorD_Dir, HIGH);
  MotorD_.write(255);
  analogWrite(PWM_D,pwm_d);
}

void turn_cw(){
  digitalWrite(MotorA_High, HIGH);
  digitalWrite(MotorA_Low, LOW);
  digitalWrite(MotorA_Dir, LOW);   // for direction cw or ccw
  MotorA_.write(255);
  analogWrite(PWM_A,pwm_a);

  digitalWrite(MotorB_High, HIGH);
  digitalWrite(MotorB_Low, LOW);
  digitalWrite(MotorB_Dir, LOW);
  MotorB_.write(255);
  analogWrite(PWM_B,pwm_b);

  digitalWrite(MotorC_High, HIGH);
  digitalWrite(MotorC_Low, LOW);
  digitalWrite(MotorC_Dir, LOW);
  MotorC_.write(255);
  analogWrite(PWM_C,pwm_c);

  digitalWrite(MotorD_High, HIGH);
  digitalWrite(MotorD_Low, LOW);
  digitalWrite(MotorD_Dir, LOW);
  MotorD_.write(255);
  analogWrite(PWM_D,pwm_d);
}

void turn_ccw(){
  digitalWrite(MotorA_High, HIGH);
  digitalWrite(MotorA_Low, LOW);
  digitalWrite(MotorA_Dir, HIGH);   // for direction cw or ccw
  MotorA_.write(255);
  analogWrite(PWM_A,pwm_a);

  digitalWrite(MotorB_High, HIGH);
  digitalWrite(MotorB_Low, LOW);
  digitalWrite(MotorB_Dir, HIGH);
  MotorB_.write(255);
  analogWrite(PWM_B,pwm_b);

  digitalWrite(MotorC_High, HIGH);
  digitalWrite(MotorC_Low, LOW);
  digitalWrite(MotorC_Dir, HIGH);
  MotorC_.write(255);
  analogWrite(PWM_C,pwm_c);

  digitalWrite(MotorD_High, HIGH);
  digitalWrite(MotorD_Low, LOW);
  digitalWrite(MotorD_Dir, HIGH);
  MotorD_.write(255);
  analogWrite(PWM_D,pwm_d);
}

void stop(){
//  digitalWrite(MotorA_Brake, LOW);
//  digitalWrite(MotorA_Dir, LOW);
  analogWrite(PWM_A, 0);
  analogWrite(PWM_B, 0);
  analogWrite(PWM_C, 0);
  analogWrite(PWM_D, 0);
}
// code end
