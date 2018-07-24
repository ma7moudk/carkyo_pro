#include <i2c_t3.h>

// ******* I2C Lidar Addresses ,,
#define  LIDARLite_ADDRESS   0x62    // Default I2C Address of LIDAR-Lite.
#define  RegisterMeasure     0x00    // Register to write to initiate ranging.
#define  MeasureValue        0x04    // Value to initiate ranging.
#define  RegisterHighLowB    0x8f    // Register to get both High and Low bytes in 1 call.

// ******* Global Variables ,,
const int stepPin  = 22;
const int dirPin   = 21;
const int encPin   = 20;
const int en       = 23;
int count          = 0;

// Pins (1) is (2) in the pcb
const int stepPin1 = 6;
const int dirPin1  = 5;
const int encPin1  = 2;
const int en1      = 7;
int count1         = 0;

int reading        = 0;
int n              = 0;
int x[50];
int x1[50];

int flagL = 0;
int flagR = 0;

void microstep();
void microstep1();
void setup()
{
  pinMode(stepPin, OUTPUT);
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(dirPin1, OUTPUT);

  pinMode(encPin, INPUT);
  pinMode(encPin1, INPUT);
  pinMode(en, OUTPUT);
  pinMode(en1, OUTPUT);

  Wire.begin(I2C_MASTER, 0x00, 19, 18);
  Wire1.begin(I2C_MASTER, 0x00, 37, 38);
  Serial.begin(115200);
  ////////////////////////////////////////////////////////////////////// first lidar on the left of the car
  digitalWrite(dirPin, HIGH);
  for (int i = 0; i < 52; i++)
  {
    if (digitalRead(encPin) == 0 )
    {
      flagL = 1;
      break;
    }
    microstep();
    delay(75);
  }

  digitalWrite(dirPin, LOW);
  for (int i = 0; i < 104; i++)
  {
    if (digitalRead(encPin) == 0 )
    {
      digitalWrite(dirPin, HIGH);
      flagL = 1;
      break;
    }
    microstep();
    delay(75);
  }

  digitalWrite(dirPin, HIGH);
  for (int i = 0; i < 200; i++)
  {
    if (digitalRead(encPin) == 0 )
    {
      digitalWrite(dirPin, HIGH);
      flagL = 1;
      break;
    }
    microstep();
    delay(75);
  }

  if (flagL == 0)
  {
    Serial.println("Sync Left failed");
    while (1);
  }
  else
  {
    digitalWrite(dirPin, HIGH);
    Serial.print("Sync Left done");
  }

  ///////////////////////////////////////////////////////////// second lidar on the right of the car
  digitalWrite(dirPin1, LOW);
  for (int i = 0; i < 52; i++)
  {
    if (digitalRead(encPin1) == 0 )
    {
      flagR = 1;
      break;
    }
    microstep1();
    delay(75);
  }

  digitalWrite(dirPin1, HIGH);
  for (int i = 0; i < 104; i++)
  {
    if (digitalRead(encPin1) == 0 )
    {
      digitalWrite(dirPin1, HIGH);
      flagR = 1;
      break;
    }
    microstep1();
    delay(75);
  }

  digitalWrite(dirPin1, LOW);
  for (int i = 0; i < 200; i++)
  {
    if (digitalRead(encPin1) == 0 )
    {
      digitalWrite(dirPin1, HIGH);
      flagR = 1;
      break;
    }
    microstep1();
    delay(75);
  }

  if (flagR == 0)
  {
    Serial.println("Sync Right failed");
    while (1);
  }
  else
  {
    digitalWrite(dirPin1, LOW);
    Serial.println("Sync Right done");
  }
  delay(2000);
}

void loop()
{
  unsigned long t = millis();
  digitalWrite(dirPin, !digitalRead(dirPin));
  digitalWrite(dirPin1, !digitalRead(dirPin1));
  getDist(0);
  for (int i = 1; i < 50; i++)
  {
    microstep2();
    delay(5);
    getDist(i);
    //microstep1();
  }

  Serial.print("L");
  Serial.print(",");
  Serial.print(digitalRead(dirPin));
  Serial.print(",");
  for (int i = 0; i < 50; i++)
  {
    Serial.print(x[i] / 100.0);
    Serial.print(',');
  }

  Serial.println();
  Serial.print("R");
  Serial.print(",");
  Serial.print(digitalRead(dirPin1));
  Serial.print(",");
  for (int i = 0; i < 50; i++)
  {
    Serial.print(x1[i] / 100.0);
    Serial.print(',');
  }
  Serial.println();

  Serial.println(millis() - t);
  delay(200);
}


int nack = 0;

int getDist(int i)
{
  uint16_t busyCounter = 0; // busyCounter counts number of times busy flag is checked, for timeout
  uint8_t  busyFlag    = 1; // busyFlag monitors when the device is done with a measurement

  while (busyFlag)
  {
    if (busyCounter > 110)
    {
      break;
    }
    Wire.beginTransmission((int)LIDARLite_ADDRESS);
    Wire.write((int)0x01);
    nack = Wire.endTransmission();
    if (nack != 0)
      Serial.println("nack 1");
    Wire.requestFrom((int)LIDARLite_ADDRESS, 1);
    if (Wire.available())
      busyFlag = Wire.read() & 0x01;
    busyCounter++;
    if (busyCounter > 100)
    {
      Serial.println("> Wire is Busy");
    }
  }

  uint16_t busyCounter1 = 0; // busyCounter counts number of times busy flag is checked, for timeout
  uint8_t  busyFlag1    = 1; // busyFlag monitors when the device is done with a measurement
  while (busyFlag1)
  {
    if (busyCounter1 > 110)
    {
      break;
    }
    Wire1.beginTransmission((int)LIDARLite_ADDRESS);
    Wire1.write((int)0x01);
    nack = Wire1.endTransmission();
    if (nack != 0)
      Serial.println("nack 2");
    Wire1.requestFrom((int)LIDARLite_ADDRESS, 1);
    if (Wire1.available())
      busyFlag1 = Wire1.read() & 0x01;
    busyCounter1++;
    if (busyCounter1 > 100)
    {
      Serial.println("> Wire is Busy");
    }
  }
  
  Wire.beginTransmission((int)LIDARLite_ADDRESS); // transmit to LIDAR-Lite
  //Serial.println("began 101");
  Wire1.beginTransmission((int)LIDARLite_ADDRESS);
  //Serial.println("began 201");
  Wire.write((int)RegisterMeasure);               // sets register pointer to  (0x00)
  Wire1.write((int)RegisterMeasure);
  Wire.write((int)MeasureValue);                  // sets register pointer to  (0x00)
  Wire1.write((int)MeasureValue);
  //Serial.println("write 101");
  //Serial.println("write 201");
  nack = Wire.endTransmission();                         // stop transmitting
  if (nack != 0)
    Serial.println("nack 001");
  //Serial.println("ended 101");
  nack = Wire1.endTransmission();
  if (nack != 0)
    Serial.println("nack 002");
  //Serial.println("ended 201");
  delay(2);                                       // 1 milli Min. delay to measure distance

  Wire.beginTransmission((int)LIDARLite_ADDRESS); // transmit to LIDAR-Lite
  //Serial.println("began 102");
  Wire1.beginTransmission((int)LIDARLite_ADDRESS);
  //Serial.println("began 202");
  Wire.write((int)RegisterHighLowB);              // sets register pointer to (0x8f)
  //Serial.println("write 102");
  Wire1.write((int)RegisterHighLowB);
  //Serial.println("write 202");
  nack = Wire.endTransmission();                         // stop transmitting
  if (nack != 0)
    Serial.println("nack 003");
  //Serial.println("ended 102");
  nack = Wire1.endTransmission();
  if (nack != 0)
    Serial.println("nack 004");
  //Serial.println("ended 202");
  delay(1);                                       // Wait 20ms for transmit
  Wire.requestFrom((int)LIDARLite_ADDRESS, 2);    // request 2 bytes from LIDAR-
  //Serial.println("req 101");
  Wire1.requestFrom((int)LIDARLite_ADDRESS, 2);
  //Serial.println("req 201");
  if (2 <= Wire.available())                      // if two bytes were received
  {
    x[i] = Wire.read();                        // receive high byte (overwrites previous reading)
    x[i] = x[i] << 8;                       // shift high byte to be high 8 bits
    x[i] |= Wire.read();                       // receive low byte as lower 8 bits
    if (x[i] < 9 )
      x[i] = 4000;
  }
  else
  {
    x[i] = 3000;
  }

  if (2 <= Wire1.available())                   // if two bytes were received
  {
    x1[i] = Wire1.read();                        // receive high byte (overwrites previous reading)
    x1[i] = x1[i] << 8;                       // shift high byte to be high 8 bits
    x1[i] |= Wire1.read();                       // receive low byte as lower 8 bits
    if (x1[i] < 9 )
      x1[i] = 4000;
  }
  else
  {
    x1[i] = 3000;
  }
}

void microstep()
{
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(2500);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(2500);
}

void microstep1()
{
  digitalWrite(stepPin1, HIGH);
  delayMicroseconds(2500);
  digitalWrite(stepPin1, LOW);
  delayMicroseconds(2500);
}

void microstep2()
{
  digitalWrite(stepPin, HIGH);
  digitalWrite(stepPin1, HIGH);
  delayMicroseconds(2500);
  digitalWrite(stepPin, LOW);
  digitalWrite(stepPin1, LOW);
  delayMicroseconds(2500);
}

