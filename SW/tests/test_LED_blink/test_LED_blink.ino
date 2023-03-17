void setup() {
  pinMode(D0, OUTPUT);  // Blue LED Beside CP2102
  Serial.begin(9600);
}
 
void loop() {
  Serial.println("Welcome To PCBoard.ca");  // display message on console
  digitalWrite(D0, HIGH);
  delay(500);
  digitalWrite(D0, LOW);
 
  delay(250);
}
