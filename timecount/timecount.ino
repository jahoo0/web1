unsigned long ret_time;
 
void setup(){
  Serial.begin(115200);
}
void loop(){
  Serial.print("Time: ");
  ret_time = (unsigned long)millis();
  //prints time since program started
  Serial.println(ret_time);
  // wait a second so as not to send massive amounts of data
  delay(1000);
}
