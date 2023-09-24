#define PIN_LED 7
unsigned int blinkcount;

void setup(){
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200); //Initialize serial port
  while (!Serial){
    ;//wait for serial port to connect.
  }

  blinkcount=0;
}

void loop(){
  digitalWrite(PIN_LED, LOW);
  delay(1000);
  while(1){
    digitalWrite(PIN_LED, HIGH);
    delay(100);
    digitalWrite(PIN_LED, LOW);
    delay(100);
    blinkcount++;

   if(blinkcount==5){
    break;}
    }

      while (1) {
   //영원히 종료시킴  

     digitalWrite(PIN_LED, HIGH);
      }
  


 
}
