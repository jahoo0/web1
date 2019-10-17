// code start
#define PIN_A2 9
#define PIN_B2 8
#define PWM 10
#define encoderA 2
#define encoderB 3

int encoderPos = 0;

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
  while(!Serial.available()); // 시리얼 입력 받을 때 까지 기다리기
  while(Serial.available()){
    val = Serial.read();
    Serial.print("input data: ");
    Serial.println(val);
    if(val=='w'){
      forward();
      Serial.println("forward");
    } else if(val=='s'){
      backward();
      Serial.println("backward");      
    } else if(val=='t'){
      stop();
      Serial.println("stop");
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
  Serial.print("A   ");
  Serial.println(encoderPos);
}

void doEncoderB(){ // 보파일 때
//  if(digitalRead(encoderPinA)==digitalRead(encoderPinB)) // 같으면
//    encoderPos--; // 역회전
//  else // 다르면
//    encoderPos++; // 정회전
  encoderPos += (digitalRead(encoderA)==digitalRead(encoderB))?-1:1;
  Serial.print("B   ");
  Serial.println(encoderPos);
}
// code end
