void setup() {
  pinMode(0, INPUT);
  pinMode(2, OUTPUT);
  Serial.begin(9600);

}

void loop() {
    if(digitalRead(0)){
      digitalWrite(2, HIGH);
      delay(100);
      digitalWrite(2, LOW);
    }
    Serial.println(digitalRead(0));
    delay(1);
}
