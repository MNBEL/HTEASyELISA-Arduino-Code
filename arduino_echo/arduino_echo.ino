
void setup(){
  Serial.begin(38400);
}

void loop(){
  if (Serial.available()){
    Serial.println(Serial.read());
    Serial.println("here");
  }
}
