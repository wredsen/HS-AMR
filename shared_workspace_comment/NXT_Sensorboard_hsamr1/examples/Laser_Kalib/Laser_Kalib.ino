// This code reads out the ADNS7550-mousechip / the analog Values from the Tri-Sensor
// and sends it to the matlab-programm
//
#include "ADNS7550.h"

//Mauspins
#define SCLK 4 // Serial clock pin on the Arduino
#define SDO 5  // Serial data (I/O) pin on the Arduino
#define SDI 3
#define NCS 2 // Chip Select-pin on the Arduino

//#define lasercurrent 2.9145             //Laserdiodenstartstrom in mA
#define lasercurrent 2.9925 //Laserdiodenstartstrom in mA

//ADNS5020 Optical1 = ADNS5020(SCLK, SDIO, NCS);
ADNS7550 Optical1 = ADNS7550(SCLK, SDI, SDO, NCS);

//die Annaeherungspolynome der Tri-Sensoren
//Frontsensor      (an A0-FRONT)
float poly0[] = {-7821.665 , 28583.986 , -40330.189 , 27764.564 , -9750.677  , 1687.286};
//Frontsidesensor  (an A1-FRONTSIDE)
float poly1[] = {-1011.178 , 4071.688 , -6404.340 , 4951.347 , -1924.777 , 330.743};
//Backsensor       (an A2-BACK)
float poly2[] = {-5501.536 , 20915.345 , -30851.243 ,  22297.395 , -8243.740 , 1514.569};
//Backsidesensor   (an A3-BACKSIDE)
float poly3[] = {-3055.609 , 12231.330  , -19215.822 , 15021.732 , -6154.977 , 1298.638};

signed long x = 0; // Variables for our 'cursor'
signed long y = 0; //

int TriAPin = 1;

int pixeldata[676]; //speichert den Wert eines jeden Pixels
int c = 0;          //Counter variable for coordinate reporting
int mode = 99;      //Modus-variable:
                    //99-> sende kontinuierlich die x&y-Position und surface quality
                    //1-> sende einmalig das Kamerabild
                    //2-> sende kontinuierlich die Spannung an Analog0 in V
                    //3-> sende die Spannung und die dazu ausgerechnete Entfernung
                    //4-> sende einmalig die Mauswerte
                    //5-> sende den Maustyp und Aufloesung
                    //6-> resette die x,y Werte der Maus
                    //7-> resette den Mauschip
                    //8-11 ->neuer Analogpin ausgtewählt
                    //12 / 13 ->erhöhe / verringere den Laserdiodenstrom
                    //21-> aktuellen Laserstrom lesen
int sensorValue;    //speichert die Werte die gerade aus einem triangulationssensor gelesen wurden
float voltage;      //-> umgerechnet in Volt
float abstand = 0;  //-> in mm
int p_grad0, p_grad1, p_grad2, p_grad3;

void setup()
{
  float temp;
  Serial.begin(38400);
  //Tri-sensors-polyfitarraysizes
  p_grad0 = sizeof(poly0) / sizeof(float);
  p_grad1 = sizeof(poly1) / sizeof(float);
  p_grad2 = sizeof(poly2) / sizeof(float);
  p_grad3 = sizeof(poly3) / sizeof(float);
  Optical1.begin();
  for (int i = 0; i < Optical1.get_px_count(); i++)
  {
    pixeldata[i] = 0;
  }

  //prozentualen startlaserstrom ausrechnen
  temp = ((lasercurrent / 5) - 0.3359) / 0.0026 - 95;
  //und einstellen
  Optical1.new_cur(temp);
}

int incomingByte = 1; // for incoming serial dataµ

void loop()
{
  //per Seriellen Commando Modus switchen
  if (Serial.available() > 0)
  {
    delay(1);

    incomingByte = Serial.parseInt();
    //Serial.println(incomingByte);

    if (incomingByte == 1)
    { //schicke das mausbild
      mode = 1;
      Serial.println("pdata=");
      Optical1.pdata(&pixeldata[0]);

      for (int i = 0; i < Optical1.get_px_count(); i++)
      {
        Serial.write(pixeldata[i]);
      }
      Serial.println();
    }
    signed int dxtemp = 0;
    signed int dytemp = 0;

    if (incomingByte == 4)
    { //schicke nur einmal die Messwerte wenn gefordert:
      mode = 4;
      if (Optical1.inmotion())
      {
        //x += Optical1.dx();                   // Read the dX register and in/decrease X with that value
        //y += Optical1.dy();                   // Same thing for dY register.....

        Optical1.dxdy(&dxtemp, &dytemp);
        x += dxtemp;
        y += dytemp;
      }

      //Serial.print("x=");
      Serial.print(x, DEC);
      Serial.print(" ");
      //Serial.print(" y=");
      Serial.println(y, DEC);
      //Serial.print(" dx=");
      //Serial.print(dxtemp, DEC);
      //Serial.print(" dy=");
      //Serial.print(dytemp, DEC);
      //Serial.print(" SQUAL=");
      //Serial.println(Optical1.squal());
    }
    else
    {
      //maustyp schicken
      if (incomingByte == 5)
      {
        Serial.print("typ=");
        Serial.print(Optical1.get_type());
        Serial.print("px=");
        Serial.println(Optical1.get_px_count());
      }
      // counter ressetten
      if (incomingByte == 6)
      {
        mode = 99;
        Optical1.dxdy(&dxtemp, &dytemp); //reset dx and dy
        x = 0;
        y = 0;
      }
      //Maus resetten
      if (incomingByte == 7)
      {
        Optical1.begin(); // Resync (not really necessary?)

        Optical1.reset();
        Optical1.forced_awake(1);
        mode = 0;
      }
      //neuen Analogpin auslesen
      if (incomingByte >= 8 && incomingByte <= 11)
      {
        TriAPin = incomingByte - 8;
      }
      //laserdiodenstrom anch oben / unten korrigieren
      if (incomingByte == 12)
      {
        Serial.print("new cur");
        Serial.println(Optical1.new_cur(3), DEC);
      }
      if (incomingByte == 13)
      {
        Serial.print("new cur");
        Serial.println(Optical1.new_cur(-3), DEC);
      }

      if (incomingByte == 21)
      {
        Serial.println("LaserCurrent:");
        Serial.println(Optical1.new_cur(0), DEC);
        mode = 4;
      }

      if (incomingByte == 99 || incomingByte == 2 || incomingByte == 3 || incomingByte == 5)
      {
        mode = incomingByte;
        //mode = 2;
      }
    }
  }

  //mode =2;
  //je nach Modus verschiedene Dinge auslesen
  switch (mode)
  {

  case 99: //sende Position und Geschwindigkeit
    if (Optical1.inmotion())
    {
      //x += Optical1.dx();                   // Read the dX register and in/decrease X with that value
      //y += Optical1.dy();                   // Same thing for dY register.....
      signed int dxtemp = 0;
      signed int dytemp = 0;

      Optical1.dxdy(&dxtemp, &dytemp);
      x += dxtemp;
      y += dytemp;
    }
    Serial.print("x=");
    Serial.print(x, DEC);
    Serial.print(" y=");
    Serial.print(y, DEC);
    Serial.print(" SQUAL=");
    Serial.println(Optical1.squal(), DEC);
    delay(200);
    break;
  case 5: //sende Position und Geschwindigkeit
    Optical1.inmotion();
    delay(300);
    break;

  case 1: //lese-mausbildpixel aus
    Optical1.pdata(&pixeldata[0]);

    break;

  case 2:
    sensorValue = analogRead(0);
    // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
    voltage = sensorValue * (5.0 / 1023.0);
    // print out the value you read:

    //Serial.print(sensorValue);
    Serial.println(voltage,4);

    delay(100);
    break;

  case 3:
    //sende kontinuierlich den Weg in mm mittels Fitfunktion errechnet:
    sensorValue = analogRead(TriAPin);
    // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
    voltage = sensorValue * (5.00 / 1023.00);
    //Spannungswert mittels fit-Funktion in mm umrechnen
    if(voltage > 1.25){
       Serial.print(voltage,3);
       Serial.println(" V");
       Serial.println("90");
       Serial.println(" ");
    delay(20);
    }
    if(voltage < 0.10){
       Serial.print(voltage,3);
       Serial.println(" V");
       Serial.println("810");
       Serial.println(" ");
    }
    
    abstand = 0;
    delay(500);
    switch (TriAPin)
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
    // print out the value you read:
    Serial.print(voltage,3);
    Serial.println(" V");
    Serial.println(abstand);
    Serial.println(" ");
    delay(20);
    
    break;

  default:
    break;
  }
}
