void setup() {
  // open a serial connection
  Serial.begin(9600);
}

void loop() {
  // read the input on A0 at default resolution (10 bits)
  // and send it out the serial connection
 // analogReadResolution(10);
  Serial.print("ADC 10-bit (default) : ");
  Serial.println(analogRead(1));

  // a little delay to not hog Serial Monitor
  delay(1000);
}
