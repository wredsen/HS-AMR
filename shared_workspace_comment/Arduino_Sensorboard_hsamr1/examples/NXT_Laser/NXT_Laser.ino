//NXT.ino - Send Data to the NXT over a RS485-Connection
#include "ADNS7550.h"
#include <math.h>

//Mousepins
#define SCLK 4 // Serial clock pin on the Arduino
#define SDO 5  // Serial data (I/O) pin on the Arduino
#define SDI 3
#define NCS 2 // Chip Select-pin on the Arduino

#define lasercurrent 2.9145 //Laserdiodenstartstrom in mA

ADNS7550 Optical1 = ADNS7550(SCLK, SDI, SDO, NCS);

//!must be the same baudrate as in the leJOS-Programm!
#define RS485_BAUD 38400

//die Annaeherungspolynome der Tri-Sensoren
//Frontsensor      (an A0-FRONT)
float poly0[] = {-782.1665 , 2858.3986 , -4033.0189 , 2776.4564 , -975.0677  , 168.7286};
//Frontsidesensor  (an A1-FRONTSIDE)
float poly1[] = {-780.1980 , 2929.3684 , -4273.5423 , 3061.6578  ,  -1121.1679  , 197.1396};
//Backsensor       (an A2-BACK)
float poly2[] = {-550.1536 , 2091.5345 , -3085.1243 ,  2229.7395 , -824.3740 , 151.4569};
//Backsidesensor   (an A3-BACKSIDE)
float poly3[] = {-305.5609 , 1223.1330  , -1921.5822 , 1502.1732 , -615.4977 , 129.8638};

// Number of samples for floating mean
const int lenght = 15;

// Arrays for floating mean
float voltage_front[lenght];
float voltage_frontside[lenght];
float voltage_back[lenght];
float voltage_backside[lenght];

// index for floating mean arrays
int index_backside = 0;
int index_back = 0;
int index_frontside = 0;
int index_front = 0;

//the Distancesensor-pins on the Arduino (Analogpin-)
const byte FRONT = 3;
const byte FRONTSIDE = 2;
const byte BACK = 1;
const byte BACKSIDE = 0;


//MausodometrieVariablen
//Mauskallibrierwerte
const int Msx = 40; //Pixel per mm
const int Msy = 40;

signed int Mdx = 0;   //bewegte Strecke
signed int Mdy = 0;   //
signed int Mdx_s = 0; //bewegte Strecke in mm
signed int Mdy_s = 0; //
signed int Mdt = 0;   //in dieser Zeitspanne

// Pins and wires
//         Arduino                Wire       MAX485
const byte RS485_OUT_PIN = 13; // yellow --> DI
const byte RS485_IN_PIN = 11;  // green  --> RO
const byte RS485_CTL_PIN = 12; // grey   --> RE+DE
//         VCC                           --> VCC
//         ------------------------------------------
//         NXT Sensor Port 4      Wire       MAX485
//         DIGIAI1                blue   --> B
//         DIGIAI0                yellow --> A
//         GND                    black  --> GND

//for speed-reasons use PORTB-outputs
//(Pin- zu Port-belegung: B 0 0 13 12 11 10 9 8
const byte RECV_RS485_BIT = B00001000;
const byte SEND_RS485_BIT = B00100000;
#define readRS485inPin (PINB & RECV_RS485_BIT)

//Distance-sensors
signed int dist1 = 1; //abstand Sensor 1 (Front)
signed int dist2 = 1; //abstand Sensor 2 (FrontSide)
signed int dist3 = 1; //abstand Sensor 3 (Back)
signed int dist4 = 1; //abstand Sensor 4 (Backside)

const unsigned int BUFLEN = 128;
byte outBuf[BUFLEN];
byte inBuf[BUFLEN];
unsigned int innbuf;

const byte SER_TIMEOUT = 7;
unsigned int RS485_LEN;
unsigned int bitDelay;
unsigned int halfBitDelay;

const byte CYCLES = 45;

// ------------------------------------------------------------------
void serSetup(unsigned long baud)
{
  pinMode(RS485_OUT_PIN, OUTPUT);
  digitalWrite(RS485_OUT_PIN, LOW);
  pinMode(RS485_IN_PIN, INPUT);
  digitalWrite(RS485_IN_PIN, LOW);
  pinMode(RS485_CTL_PIN, OUTPUT);
  digitalWrite(RS485_CTL_PIN, LOW);

  RS485_LEN = 1000000 / baud;
  bitDelay = RS485_LEN - clockCyclesToMicroseconds(CYCLES);
  halfBitDelay = RS485_LEN / 2 - clockCyclesToMicroseconds(CYCLES);
  Serial.print("\nRS-485 configured, ");
  Serial.print(baud, DEC);
  Serial.print(" Baud, Bit length ");
  Serial.print(RS485_LEN, DEC);
  Serial.println(" uSec");
}

// Startbit of 1st byte of incoming message encountered
// ------------------------------------------------------------------
byte serAvail()
{
  return readRS485inPin;
}

// Read a byte
// ------------------------------------------------------------------
unsigned int serRead()
{
  byte i;
  int val = 0;
  byte parity = 0, stopbit = 0;

  long start = millis();
  while (true)
  {
    if (millis() - start > SER_TIMEOUT)
      return -1;
    if (readRS485inPin)
      break; // startbit hit
  }

  delayMicroseconds(halfBitDelay); // jump to middle of startbit
  for (i = 0; i < 8; i++)
  {
    delayMicroseconds(bitDelay);
    val |= !readRS485inPin << i; // data bits
  }
  //delayMicroseconds(bitDelay);
  //parity = !readRS485inPin;        //
  delayMicroseconds(bitDelay);
  stopbit = !readRS485inPin; //

  return val;
}

// Write one bit
// -----------------------------------------------------------------
void serBit(byte mark)
{
  if (mark)
    PORTB &= ~SEND_RS485_BIT;
  else
    PORTB |= SEND_RS485_BIT;
  //  delayMicroseconds(RS485_LEN);
  delayMicroseconds(bitDelay);
}

// Write a byte
// ------------------------------------------------------------------
void serWrite(byte data)
{
  byte mask = 1;
  byte bitcount = 0;

  serBit(LOW); // startbit
  for (byte i = 0; i < 8; i++)
  { // data from LSB to MSB
    if (data & mask)
    {
      serBit(HIGH);
      bitcount++;
    }
    else
    {
      serBit(LOW);
    }
    mask <<= 1;
  }
  //serBit((bitcount%2)==0);   // odd parity bit
  serBit(HIGH); // stop bit
}

// ------------------------------------------------------------------
void printMsg(const char *s, byte buf[], int nbuf)
{
  Serial.print(s);
  Serial.print(": \"");
  for (int i = 0; i < nbuf; i++)
  {
    Serial.print(buf[i]);
    Serial.print(",");
    delayMicroseconds(10);
  }
  Serial.println("\"");
}

// ------------------------------------------------------------------
void serSendMsg(byte buf[], int nbuf)
{
  int i;

  digitalWrite(RS485_CTL_PIN, HIGH);
  for (i = 0; i < nbuf; i++)
    serWrite(buf[i]);
  digitalWrite(RS485_CTL_PIN, LOW);
}

// ------------------------------------------------------------------
int serRecvMsg(byte buf[])
{
  int c;
  int nbuf = 0;

  while ((c = serRead()) != -1)
    buf[nbuf++] = c;
  return nbuf;
}

// ------------------------------------------------------------------
void measureComms(byte stage)
{
  static unsigned long old = 0;
  unsigned long now;

  if (!stage)
  {
    old = millis();
  }
  else
  {
    now = millis();
    Serial.print("comms took ");
    Serial.print(now - old, DEC);
    Serial.println(" mSec");
  }
}
int sensorValue;   //speichert die werte die gerade aus einem triangulationssensor gelesen wurden
float voltage;     //-> umgerechnet in Volt
float abstand; //-> in mm
int p_grad0, p_grad1, p_grad2, p_grad3;

// ------------------------------------------------------------------
int getAbstand(int readPin)
{

  //get the mean of 15 samples from the analog signal from readPin in V
  voltage = calculateMean(readPin);

  //limit for the voltage out of the calibrated frame 
  if(voltage > 1.25){
      return 9;
    }
    
  if(voltage < 0.10){
      return 81;
    }


  //Spannungswert mittels fit-Funktion in cm umrechnen
  abstand = 0;
  switch (readPin)
  {
  case 0:
    for (int i = 0; i < p_grad0; i++)
    {
      abstand += poly0[i] * pow(voltage, (p_grad0 - i - 1));
    }
    break;
  case 1:
    for (int i = 0; i < p_grad1; i++)
    {
      abstand += poly1[i] * pow(voltage, (p_grad1 - i - 1));
    }
    break;
  case 2:
    for (int i = 0; i < p_grad2; i++)
    {
      abstand += poly2[i] * pow(voltage, (p_grad2 - i - 1));
    }
    break;
  case 3:
    for (int i = 0; i < p_grad3; i++)
    {
      abstand += poly3[i] * pow(voltage, (p_grad3 - i - 1));
    }
    break;
  }
   //print out the value you read:
   //Serial.println("V");
   //Serial.println(voltage);
   //Serial.println(abstand);
   
   //limit for calculated distance 
   if(abstand < 10){
      return 10;
   }
   if(abstand > 80){
      return 80;
   }

  //symetrical round befor cast to int
  int dis = round(abstand);

  //return result
  return dis;
}
// ------------------------------------------------------------------

// fill the arrays for the mean calculation with values for begin to avoid faulty results
void setupSharp()
{

  for (int i = 0; i < 4; i++)
  {

    for (int j = 0; j < lenght; j++)
    {

      sensorValue = analogRead(i);
      // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
      voltage = sensorValue * (5.0 / 1023.0);

      if (i == BACKSIDE)
      {
        voltage_backside[j] = voltage;
      }
      if (i == BACK)
      {
        voltage_back[j] = voltage;
      }
      if (i == FRONTSIDE)
      {
        voltage_frontside[j] = voltage;
      }
      if (i == FRONT)
      {
        voltage_front[j] = voltage;
      }
    }
  }
}

// ------------------------------------------------------------------
//function to calculate the mean of 15 samples

float calculateMean(int readPin)
{

  // temporary variable to calculate mean value
  float mean;

  //messe und errechne kontinuierlich den Weg
  sensorValue = analogRead(readPin);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  voltage = sensorValue * (5.0 / 1023.0);

  switch (readPin)
  {

  case BACKSIDE:

    //write new sample to the array
    voltage_backside[index_backside] = voltage;
    index_backside++;

    //itterate thru the array and add values of
    for (int i = 0; i < lenght; i++)
    {
      mean += voltage_backside[i];
    }
    //calculate moving index
    index_backside = index_backside % lenght;

    //return mean
    return mean / lenght;

    break;

  case BACK:

    voltage_back[index_back] = voltage;
    index_back++;

    for (int i = 0; i < lenght; i++)
    {
      mean += voltage_back[i];
    }
    index_back = index_back % lenght;
    return mean / lenght;

    break;

  case FRONTSIDE:

    voltage_frontside[index_frontside] = voltage;
    index_frontside++;

    for (int i = 0; i < lenght; i++)
    {
      mean += voltage_frontside[i];
    }
    index_frontside = index_frontside % lenght;
    return mean / lenght;

    break;
    
  case FRONT:

    voltage_front[index_front] = voltage;
    index_front++;

    for (int i = 0; i < lenght; i++)
    {
      mean += voltage_front[i];
    }
    index_front = index_front % lenght;
    return mean / lenght;

    break;
  }
}

// ------------------------------------------------------------------
void setup()
{
  float temp;
  //PC-Connection  (comment out for max speed)
  Serial.begin(38400);
  //MAX485 Connection
  serSetup(RS485_BAUD);
  //Maussetup
  Optical1.begin(); // Resync (not really necessary?)
  //Optical1.reset();
  //Optical1.forced_awake(1);            //set Mouse to always-on-mode
  //Tri-sensors-polyfitarraysizes
  p_grad0 = sizeof(poly0) / sizeof(float);
  p_grad1 = sizeof(poly1) / sizeof(float);
  p_grad2 = sizeof(poly2) / sizeof(float);
  p_grad3 = sizeof(poly3) / sizeof(float);

  //prozentualen startlaserstrom ausrechnen
  temp = ((lasercurrent / 5) - 0.3359) / 0.0026 - 95;
  //und einstellen
  Optical1.new_cur(temp);
  setupSharp();
}

// ------------------------------------------------------------------

void loop()
{
 // measureComms(0);
  if (serAvail()) {
    innbuf = serRecvMsg(inBuf);
    if(inBuf[0] == 23)  //Anfrage nach Sensordaten
    {
      //holen der Mauswerte
      Mdx_s = (Mdx/Msx);
      Mdy_s = (Mdy/Msy);     
      
      Mdt = millis()-Mdt;  //theoretisch immer 100 ms
      
      //debugreport to PC
    /*  Serial.println(Mdx_s);    //comment out for maxspeed
      Serial.println(Mdy_s);    //comment out for maxspeed     
      Serial.println(Mdt);      //comment out for maxspeed
      */
      //messen der Tri-sensorwerte
       dist1 = getAbstand(FRONT);
       dist2 = getAbstand(FRONTSIDE);
       dist3 = getAbstand(BACK);
       dist4 = getAbstand(BACKSIDE);
       
       //Odometrie
       outBuf[0] = (byte) (Mdx_s & 0xFF);
       outBuf[1] = (byte) (Mdx_s >> 8);
       outBuf[2] = (byte) (Mdy_s & 0xFF);
       outBuf[3] = (byte) (Mdy_s >> 8);
       outBuf[4] = (byte) (Mdt & 0xFF);
       outBuf[5] = (byte) (Mdt >> 8);
       //DistanceSensors
       outBuf[6] = (byte) (dist1 & 0xFF);
       outBuf[7] = (byte) (dist1 >> 8);
       outBuf[8] = (byte) (dist2 & 0xFF);
       outBuf[9] = (byte) (dist2 >> 8);
       outBuf[10] = (byte) (dist3 & 0xFF);
       outBuf[11] = (byte) (dist3 >> 8);
       outBuf[12] = (byte) (dist4 & 0xFF);
       outBuf[13] = (byte) (dist4 >> 8);
       
       //mauszaehler zuruecksetzen
       Mdx =0;
       Mdy = 0;
       Mdt = millis();
       //send the Sensorvalues+time
       serSendMsg(outBuf, 14);
       // measureComms(1);
    }
    //Mauswerte jeden Takt aktuallisieren
    if(Optical1.inmotion()){
      Optical1.dxdy(&Mdx, &Mdy);
      //Mdy += Optical1.dy();
    }
  
   // printMsg("Recv", inBuf, innbuf);
   // printMsg("Sent", outBuf, 13);

  }
}