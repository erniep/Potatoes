// Pin 13 has an LED connected on most Arduino boards.
// give it a name:
int led = 13;
int lightSensor = 8;

// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  Serial.begin(9600);
  pinMode(led, OUTPUT);
  pinMode(lightSensor, OUTPUT);
}

// the loop routine runs over and over again forever:

int lightValue;
int counter = 100;
void loop() {
  pinMode(lightSensor,OUTPUT);
  digitalWrite(lightSensor, HIGH);
  delay(10);                // 0.5 sec delay
  pinMode(lightSensor, INPUT);
  digitalWrite(lightSensor, LOW);
  delayMicroseconds(0 + (counter));
  lightValue = digitalRead(lightSensor);
  
  if (lightValue = HIGH) {
    digitalWrite(led, HIGH);
  }  else {
    digitalWrite(led, LOW);
  }
  
  Serial.println(counter);
  Serial.println(":");
  Serial.println(lightValue);
  Serial.println("-----");
  delay(1000);
  
  counter += 100;
  
  if (counter > 10000) {
    counter = 0;
  }
  /*long result = 0;
   pinMode(lightSensor, OUTPUT);       // make pin OUTPUT
   digitalWrite(lightSensor, HIGH);    // make pin HIGH to discharge capacitor - study the schematic
   delay(10);                       // wait a  ms to make sure cap is discharged

   pinMode(lightSensor, INPUT);        // turn pin into an input and time till pin goes low
   digitalWrite(lightSensor, LOW);     // turn pullups off - or it won't work
   while(digitalRead(lightSensor)){    // wait for pin to go low
      result++;
   }

  Serial.println(result);
  
  delay(500);*/
}
