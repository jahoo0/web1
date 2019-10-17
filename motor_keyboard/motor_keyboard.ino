// code start
#define PIN_A2 9
#define PIN_B2 8
#define PWM 10

char val;

void setup() {
  pinMode(PIN_A2, OUTPUT);
  pinMode(PIN_B2, OUTPUT);
  Serial.begin(9600);
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
  analogWrite(PWM, 150);
}

void backward(){
  digitalWrite(PIN_A2, LOW);
  digitalWrite(PIN_B2, HIGH);
  analogWrite(PWM, 150);
}

void stop(){
  digitalWrite(PIN_A2, LOW);
  digitalWrite(PIN_B2, LOW);
  analogWrite(PWM, 0);
}
// code end
