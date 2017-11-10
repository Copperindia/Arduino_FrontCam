#include <SoftwareSerial.h>
SoftwareSerial BTSerial(2, 3); //블루투스센서 포트 설정
#define sensorPin 0
#define VOLTS_PER_UNIT    .0049F        // (.0049 for 10 bit A-D)
#define SHOCK 8 // 충격감지 센서 핀 설정
const int buzzerPin= 9; //부저모듈
float volts;
float inches;
float proxSens = 0;
int cm;
int cntStair=0;
int cntWall=0;
int trigPinTop = 12;
int echoPinTop = 13;
int trigPinBot = 11;
int echoPinBot = 10;


long durationTop, distanceTop, durationBot, distanceBot;
void checkBarrier(int distanceTop, int distanceBot);
unsigned long start, finished, elapsed;

void setup()  
{
  Serial.begin(9600);
  pinMode(sensorPin, INPUT);
  pinMode(SHOCK, INPUT);
  pinMode(trigPinTop, OUTPUT);
  pinMode(echoPinTop, INPUT);
  pinMode(trigPinBot, OUTPUT);
  pinMode(echoPinBot, INPUT);
  pinMode(buzzerPin,OUTPUT);
  Serial.begin(9600);
  BTSerial.begin(9600);  // set the data rate for the BT port
}
 
void loop()
{
  //초음파
  digitalWrite(trigPinTop, LOW); // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPinTop, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPinTop, LOW);
 
  durationTop = pulseIn(echoPinTop, HIGH);
 
  digitalWrite(trigPinBot, LOW);
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPinBot, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPinBot, LOW);
  durationBot = pulseIn(echoPinBot, HIGH);
 
  distanceTop = (durationTop/2)/29.1;
  distanceBot = (durationBot/2)/29.1;
 
  checkBarrier(distanceTop, distanceBot);
  delay(200);

  //레이저
  proxSens = analogRead(sensorPin);
  volts = (float)proxSens * VOLTS_PER_UNIT; // ("proxSens" is from analog read)
  inches = 23.897 * pow(volts,-1.1907); //calc inches using "power" trend line from Excel
  cm = 60.495 * pow(volts,-1.1904);     // same in cm
  if (volts < .2) inches = -1.0;        // out of range    
  Serial.println(cm);
  BTSerial.println(cm);
  delay(200);
  
  //인식하면 안으로 들어감
  if(distanceTop && distanceBot<= 450){
    if(cm <= 300){
      tone(buzzerPin,349, 500); 
      delay(500);
    // Serial ?> Data ?> BT
      start=millis();
      BTSerial.write("danger");
      BTSerial.println("");
      if(digitalRead(SHOCK) == HIGH){
        finished=millis();
        elapsed=finished-start;
        if(elapsed<=500){
          Serial.println("Collision");
          BTSerial.println("Collision");
        }
      }
      delay(1000);
    }
  }
}

//초음파 센서 출력
void checkBarrier(int distanceTop, int distanceBot)    // 1 : not, 2 : stair, 3: wall
{
  // For the ocha
  
  Serial.print(distanceTop);
  Serial.print("  ");
  Serial.println(distanceBot);
  BTSerial.print(distanceTop);
  BTSerial.print("  ");
  BTSerial.println(distanceBot);

}
