#define LED_PIN 13
#define ON_PIN 7 // flash button pin
#define OFF_PIN 2 // turn off button pin
#define DELAY_MS 500 // blinking delay

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  pinMode(ON_PIN, INPUT);
  pinMode(OFF_PIN, INPUT);
}

void loop()
{
  short on = digitalRead(ON_PIN) == HIGH;
  short off = digitalRead(OFF_PIN) == HIGH;
  
  if(on && !off){
    digitalWrite(LED_PIN, HIGH);
  }else if(off && !on){
    digitalWrite(LED_PIN, LOW);
  }else{
  	digitalWrite(LED_PIN, HIGH);
  	delay(DELAY_MS);
  	digitalWrite(LED_PIN, LOW);
  	delay(DELAY_MS);
  }
}
