//servo1(8) + Serial3 >> right
//servo2(9) + serial2 >> left

#include <Servo.h>

Servo myservo1;
Servo myservo2;

const int range = 30;
const int Step = 2;
const int l = (range/Step)+1;
float dist1[l];
float dist2[l];

void setup()
{
  pinMode(9,OUTPUT);
  pinMode(8,OUTPUT);
  myservo1.attach(8);
  myservo2.attach(9);
  myservo1.write(90);
  myservo2.write(90);
  delay(500);
  myservo1.write(90-(range/2));
  myservo2.write(90-(range/2));
  Serial.begin(115200); // Serial output through USB to computer
  Serial3.begin(115200); // Serial output through USB to computer
  Serial2.begin(115200); // Serial output through USB to computer
  delay(100); // Give a little time for things to start
}

int count=0;
unsigned long period=millis();

void loop()
{
  unsigned long int t2=millis();

  for(int pos = 0; pos <= range; pos += Step) 
  {
    myservo1.write(90-pos);
    myservo2.write(90+pos);
    delay(17);
    unsigned long int t = millis();
    
    float d1 = readLiDAR1()/100.0;
    if(d1 < 0.1)
      d1 = 30.0;
    else if (d1 > 8.0)
      d1 = 30.0;
    dist1[(l-1)-count] = d1;
    
    float d2 = readLiDAR2()/100.0;
    if(d2 < 0.1)
      d2 = 30.0;
    else if (d2 > 8.0)
      d2 = 30.0;
    dist2[count] = d2;
    count++;

    while (millis()-t < 10);
  }
  
  Serial.print("R,");
  for (int i=0; i<l; i++)
  {
    Serial.print(dist1[i]);
    Serial.print(",");
  }
  Serial.println();
  
  Serial.print("L,");
  for (int i=0; i<l; i++)
  {
    Serial.print(dist2[i]);
    Serial.print(",");
  }
  Serial.println();
  
  myservo1.write(90);
  myservo2.write(90);
  delay(500);
  count=0;
  Serial.println(millis()-t2);

}

unsigned int readLiDAR1()
{
  unsigned int liDARval=0;
  while(!Serial3.available())   
    Serial.println("waiting"); // wait for a character

  while(Serial3.available()>16) // When at least 9 bytes of data available (expected number of bytes for 1 signal), then read
  {
    int z = Serial3.read();
   // Serial.println("clearing");
 }  
  while(Serial3.available()>=9 ) // When at least 9 bytes of data available (expected number of bytes for 1 signal), then read
  {
    if((0x59 == Serial3.read()) && (0x59 == Serial3.read())) // byte 1 and byte 2
    {
      unsigned int t1 = Serial3.read(); // byte 3 = Dist_L
      unsigned int t2 = Serial3.read(); // byte 4 = Dist_H     
      t2 <<= 8;        
      t2 += t1;
      liDARval = t2;

      t1 = Serial3.read(); // byte 5 = Strength_L
      t2 = Serial3.read(); // byte 6 = Strength_H

      t2 <<= 8;
      t2 += t1;
      for(int i=0; i<3; i++)
        Serial3.read(); // byte 7, 8, 9 are ignored
    }
  }
  return liDARval;
}

unsigned int readLiDAR2()
{
  unsigned int liDARval=0;
  while(!Serial2.available())   
    Serial.println("waiting"); // wait for a character

  while(Serial2.available()>10) // When at least 9 bytes of data available (expected number of bytes for 1 signal), then read
  {
    int z = Serial2.read();
   // Serial.println("clearing");

 }  
  while(Serial2.available()>=9 ) // When at least 9 bytes of data available (expected number of bytes for 1 signal), then read
  {

    if((0x59 == Serial2.read()) && (0x59 == Serial2.read())) // byte 1 and byte 2
    {
      unsigned int t1 = Serial2.read(); // byte 3 = Dist_L
      unsigned int t2 = Serial2.read(); // byte 4 = Dist_H     
      t2 <<= 8;        
      t2 += t1;
      liDARval = t2;

      t1 = Serial2.read(); // byte 5 = Strength_L
      t2 = Serial2.read(); // byte 6 = Strength_H

      t2 <<= 8;
      t2 += t1;
      for(int i=0; i<3; i++)
        Serial2.read(); // byte 7, 8, 9 are ignored
    }
  }
  return liDARval;
}
