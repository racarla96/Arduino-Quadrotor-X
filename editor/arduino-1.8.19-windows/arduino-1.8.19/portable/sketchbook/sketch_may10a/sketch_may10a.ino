volatile unsigned long previousTime = 0;
volatile unsigned long elapsedTime = 0;

void setup() {
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(3), isr, RISING);
}

void loop() {
  if(elapsedTime > 0){
    Serial.print("Elapsed time: ");
    Serial.println(elapsedTime);
    elapsedTime = 0;
  }
}

void isr() {
  unsigned long currentTime = micros();
  if(previousTime != 0){
    elapsedTime = currentTime - previousTime;
  }
  previousTime = currentTime;
}
