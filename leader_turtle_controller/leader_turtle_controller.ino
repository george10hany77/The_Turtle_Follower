void setup() {
  Serial.begin(115200);
}

void loop() {
  int x_val = map(analogRead(A0), 0, 1023, -350, 373);
  int y_val = map(analogRead(A1), 0, 1023, -365, 352);
  Serial.println("(" + String(x_val) + "," + String(y_val) + ")");
}
