void setup() {
  // put your setup code here, to run once:
  delay(500);
  Serial2.begin(9600);  //9600
  delay(500);
  Serial2.print("AT+NAMEZJN");  //do not use \r\n
  delay(500);
  Serial2.println("AT+BAUD8");  //8 refer to 115200
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("ok...\n");
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000); 
}
