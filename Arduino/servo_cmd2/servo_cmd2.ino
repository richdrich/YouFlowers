// This is the unbuzzy version
// Turns the servo off when not actually being rotated
// (having worked out the rotate time)

#include <Servo.h>

int servoPins[] = {10,11,A2,A3,A4,A5};  
int blinkPin = 2;
#define PIN_CYCLES 200000L

Servo myservo[6];  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 
 
int lastPos[6] = {360,360,360,360,360,360};


void setup() 
{ 
  for(int n=0; n<6; n++) {
    myservo[n].attach(servoPins[n]);  // attaches the servo on each pin to the servo object
  }
  
  Serial.begin(9600);
  Serial.println("Reset");
  
  pinMode(blinkPin, OUTPUT);
} 
 

void loop() 
{ 
  Serial.write("Ok>");

  int rotateTime;
  char * msg = readMsg();
  if(msg[1]=='X') {
    for(int n=0; n<6; n++) {
      setServo(n, 90);
    }
    
    Serial.println("Centre");
    return;
  }
  
  char servoNum = msg[1] - '0';
  const int maxAng = 170;
  int degs = atoi(&msg[2]);
  if(degs < 10) degs=10;
  if(degs >= maxAng) degs=maxAng;
  
  setServo(servoNum, degs);
}

void setServo(int servoNum, int degs) {

  int rotateTime = (abs(degs-lastPos[servoNum]) * 10) / 3;  // 600ms / 180deg = 10*deg / 3

  myservo[servoNum].attach(servoPins[servoNum]);
  myservo[servoNum].write(degs);
  delay(rotateTime);
  myservo[servoNum].detach();
  lastPos[servoNum] = degs;
  
}

char * readMsg() {
  static char buf[7];
  char pinState;
  long pinCount;
  
  char * bp = buf;
  
  do {
    pinState=0;
    pinCount=PIN_CYCLES;
    while(!Serial.available()) {
      pinCount--;
      if(pinCount==0) {
        pinCount=PIN_CYCLES;
        pinState = (pinState+1) % 2;
        digitalWrite(blinkPin, pinState);
      }
    }
    
    char c = Serial.read();
    if(c=='\r' || c=='\n') {
      if(buf[0] == 'S') {
        *bp = 0;
        Serial.write("\r\n");
        return buf;
      }
      
      Serial.write("?");
      bp = buf;
      continue;
    }
    
    if(bp > &buf[6]) {
      bp = buf;
    }
    *bp++ = c;
    Serial.write(c);
  }
  while(true);
}


