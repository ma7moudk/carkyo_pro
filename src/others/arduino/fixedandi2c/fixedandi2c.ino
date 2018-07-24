//middle

#include <Wire.h>
#include <LIDARLite.h>
#include <Servo.h>

LIDARLite myLidarLite;
Servo myservo;
Servo myservo2;

const int range = 80;
const int Step = 5;
const int l = (range/Step)+1;
float dist[l];

void setup()
{
  myLidarLite.begin(0, true); 
  myLidarLite.configure(0);
  pinMode(8,OUTPUT);

  myservo.attach(8); 
  
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);
  myservo.write(90-(range/2));

  delay(1000);
  Serial.println("setup");
}

int count=0;
void loop()
{

  unsigned long int t2=millis();
  
  for(int pos = 90-(range/2); pos <= 90+(range/2); pos += Step) 
  {
//Serial.println(pos);
    myservo.write(pos);
    delay(30);
    unsigned long int t=millis();
    float d=myLidarLite.distance()/100.0;
//    Serial.println(d);
    if(d<0.1)
      d=30.0;
    else if (d>30.0)
      d=30.0;
    dist[count] = d;
    count++;
   
    while (millis()-t < 10);
  }
  count=0;
  Serial.print("1st");
  for (int i=0; i<l; i++)
  {
    Serial.print(",");
    Serial.print(dist[i]);
  }
  Serial.println();
  myservo.write(90-(range/2));
  delay(400);
  Serial.println(millis()-t2);
}
