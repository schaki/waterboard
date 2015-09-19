const int pingPin = 6;
int calibrationTime = 30;
long unsigned int lowIn;
long unsigned int pause = 5000;
boolean lockLow = true;
boolean takeLowTime;

int pirPin = 3;    //the digital pin connected to the PIR sensor's output
int ledPin = 9;

#define DATA_PIN (11)
#define CLOCK_PIN (13)
#define MULTIPLE (5)
#define LIGHT_COUNT (8)

uint32_t colors[] = {
  0x00FF0000,
  0x0000FF00,
  0x00000000,
  0x000000FF
};

uint32_t pixels[LIGHT_COUNT];

#define pinModeFast(x, y) pinMode(x, y)
#define digitalWriteFast(x, y) digitalWrite(x, y)

void setup() {
  //Serial.begin(9600);
  pinModeFast(DATA_PIN, OUTPUT);
  pinModeFast(CLOCK_PIN, OUTPUT);
  digitalWriteFast(DATA_PIN, LOW);
  digitalWriteFast(CLOCK_PIN, LOW);
  pinMode(pirPin, INPUT);
  pinMode(ledPin, OUTPUT);
  digitalWrite(pirPin, LOW);

  //give the sensor some time to calibrate
  /*Serial.print("calibrating sensor ");
    for(int i = 0; i < calibrationTime; i++){
      Serial.print(".");
      delay(1000);
      }
    Serial.println(" done");
    Serial.println("SENSOR ACTIVE");
    delay(50);*/
}

static void set_pixel(uint8_t index, uint32_t color) {
  pixels[index] = color & 0x00FFFFFF;
}

static void show() {
  digitalWriteFast(DATA_PIN, LOW);
  for (int i = 0; i < 50; i++) {
    toggle_clock();
  }
  for (int i = 0; i < LIGHT_COUNT; ++i) {
    write_pixel(i);
  }
  write_blank_pixel();
  delay(1);
}

static void toggle_clock() {
  digitalWriteFast(CLOCK_PIN, HIGH);
  digitalWriteFast(CLOCK_PIN, LOW);
}

static void write_pixel(uint8_t i) {
  const uint32_t MASK = ((uint32_t)(1) << 24);
  uint32_t p = pixels[i] | MASK;
  int j = 25;
  while (j--) {
    digitalWriteFast(DATA_PIN, (p & MASK) ? HIGH : LOW);
    toggle_clock();
    p <<= 1;
  }
}

static void write_blank_pixel() {
  int j = 25;
  while (j--) {
    digitalWriteFast(DATA_PIN, 0);
    toggle_clock();
  }
}

long microsecondsToInches(long microseconds) {
  return microseconds / 74 / 2;
}

void loop() {

  /*if(digitalRead(pirPin) == HIGH){
   digitalWrite(ledPin, HIGH);   //the led visualizes the sensors output pin state
   if(lockLow){
     //makes sure we wait for a transition to LOW before any further output is made:
     lockLow = false;
     Serial.println("---");
     Serial.print("motion detected at ");
     Serial.print(millis()/1000);
     Serial.println(" sec");
     delay(50);
     }
     takeLowTime = true;
   }

 if(digitalRead(pirPin) == LOW){
   digitalWrite(ledPin, LOW);  //the led visualizes the sensors output pin state

   if(takeLowTime){
    lowIn = millis();          //save the time of the transition from high to LOW
    takeLowTime = false;       //make sure this is only done at the start of a LOW phase
    }
   //if the sensor is low for more than the given pause,
   //we assume that no more motion is going to happen
   if(!lockLow && millis() - lowIn > pause){
       //makes sure this block of code is only executed again after
       //a new motion sequence has been detected
       lockLow = true;
       Serial.print("motion ended at ");      //output
       Serial.print((millis() - pause)/1000);
       Serial.println(" sec");
       delay(50);
       }
   }  */


  long duration, inches;

  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);

  inches = microsecondsToInches(duration);

  Serial.print(inches);
  Serial.println();

  if (LIGHT_COUNT * MULTIPLE > inches) {
    for (int i = 0; i < LIGHT_COUNT; i++) {
      if (inches > i*MULTIPLE && inches < (i+1)*MULTIPLE) {
        set_pixel(i,colors[0]);
      } else {
        set_pixel(i,colors[1]);
      }
    }
  } else {
    for (int i = 0; i < LIGHT_COUNT; i++) {
        set_pixel(i,colors[2]);
    }
  }

  show();
  delay(200);

}
