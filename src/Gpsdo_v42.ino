/***********************************************************************************************************
 *        Version 4.2 ----- GPSDO -----  Fout (Rx GPS) = 100 KHz pour NEO M7 et M8
 *               
 *        Cette version inclus :
 *        - afficheur LED de type ( 4x16 ou 4x20 ) mais des informations affichées limitées
 *        ou
 *        - afficheur OLED ( 128x64 ) en 0.96" ou 1.3"
 *        
 *        - l'affichage du nombre de satelittes utilisés ( GPS + Glonass )
 *        - l'affichage du nombre de satelittes vus ( GPS + Glonass )
 *        - l'affichage de la varaible FIX 
 *        
 *        - le " Lock detect status " de la boucle de phase ( foncrion " read_PLL " )
 *          avec l'affichage de la tension de boucle ( x,xxx V )
 *          
 *        - le choix d'afichage entre Locator sur 6 digits ou Locator MH sur 10 digits
 *        
 *        Alain Fort F1CJN  November, 11, 2016     /   F4HSP  08 Octobre  2021
 * 
 * *********************************************************************************************************
 * 
 * This program is made :
 * - first to program the Timepulse frequency of an UBLOX GPS NEO-8M module
 * - latitude, longitude, Time, 
 * - number of used satellites, number of received satellites (GPS + Glonass), FIX and the QRA Locator.
 * - PLL Lock detect status (led)
 *
 * The frequency can be changed near line 125.
 * The Timepulse is based on a master clock at 48 MHz
 * Only frequencies with even sub multiples integer numbers (ie, 2,4,6, 342 .....) have low jitter. For exemple :8MHz is ok(6) ,10Khz is 
 * ok(4800). 10 MHz is not ok(divison factor of 4.8). The Timepulse frequency is programmed for 10 KHz and can be modified (on line of program).
 * 
 * La fréquence peut être modifiée en ligne 125/
 * The signal Timepulse est généré à partir d'une horloge à 48 MHz.
 * Seules les facteurs de divisions entiers pairs (2,4,6, 342, ....) présentent un très faible jitter. Par exemple 8MHz est ok, 10 KHz est ok 
 * mais 10 MHz ne l'est pas (facteur de division de 4,8). La fréquence de Timepulse est programmé pour 10 KHz mais peut être modifiée à la ligne 125.
 * 
 * Connections to the GPS board
 * GPS GND connects to Arduino Gnd
 * GPS RX connects to Software Serial pin 3 on the Arduino Nano, via voltage divider : 330 Ohms in serial to 680 Ohms to ground        Pin 4--330--NEO RX--680--GND
 * GPS TX connects to Software Serial pin 4 on the Arduino Nano
 * GPS VCC connects to 5 volts
 * 
 * Connections to the serial LCD 4x16 board
 * Arduino 5V to LCD 5V
 * Arduino GND to LCD GND
 * Arduino A4 to LCD SDA
 * Arduino A5 to CLD SCL
 * 
 * Serial O/P at 9600 ( or 115200 ) is used for test purposes 
 * 
 * et il faut compiler ce code avec la librairie TiniGPSPlus2 ( .h et .cpp )
 * 
 **************************************************************************/

#include <SoftwareSerial.h>
#include <TinyGPSPlus2.h>          // Attention Librairie modifiée (voir texte ci-dessus)
#include <Wire.h>                 // Comes with Arduino IDE
#include <LiquidCrystal_I2C.h>


// Librairie et Init OLED -------------- version 1.3 pouce ------------------
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
#define I2C_ADDRESS 0x3C  // 0x3C ou 0x78 
#define RST_PIN -1
SSD1306AsciiWire oled;    // 

/*
// Librairie et Init OLED -------------- version 0.96 pouce ------------------
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
#define I2C_ADDRESS 0x3C
#define RST_PIN -1
SSD1306AsciiAvrI2c oled;
*/

// LCD init pour afficheur 4 x 20
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol

// LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);        // ----- Set the LCD I2C address 0x27
//LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);        // ----- Set the LCD I2C address 0x3F
LiquidCrystal_I2C lcd(0x27,20,4);

boolean gpsStatus[] = {false, false, false, false, false, false, false, false, false};
unsigned long frequency;
byte buf[4];

int satvustotal;    // Nombre total de satellites vus par GPS + Glonass

// Communication GPS
#define RXPIN 4       // vers GPS TX
#define TXPIN 3       // vers GPS RX via pont diviseur 330/680 Ohms pour adapter en 3,3V
#define GPSBAUD 9600  //
SoftwareSerial gpsSerial(RXPIN,TXPIN);    // 4,3
TinyGPSPlus2 gps;

// Traitement tension boucle de phase
byte LED_loop = 6;                        // I/O Pin D6
byte Port_anal_loop = 0;                  // -- Input A0  
int val_loop;                             // 
int moy_val_loop;                         // 
int old_moy_val_loop;                     //
int seuil_loop = 11;                      // fenêtre de 100mV, soit plus et moins ((5 / 1024) * 11) = 0.050 V 
float U_loop;                             // tension U de la boucle de phase 

void getgps(TinyGPSPlus2 &gps);

// Traitement Latitude, longitude, locator, locator MH
float latitude;
float longitude;
char locator[6];    // locator en  6 digits
char MH[10];        // locator en 10 digits

void setup()
{
  pinMode(LED_loop, OUTPUT);      // 
  digitalWrite(LED_loop, LOW);    // ----- turn Off the LED_loop  
      
  gpsSerial.begin(9600);    // GPS à 9600 Bauds
  Serial.begin(9600);       // Serial monitor  via USB at 9600 bauds ( au lieu de 115200 bauds )

  frequency=100000;         // 100 KHz  Timepulse Frequency in Hertz. The duty cycle is fixed at 50%
                            // only frequencies with even sub multiples numbers (ie, 2,4,6, 342 .....) are not noisy   for exemple :8MHz is ok, 10Khz is ok,
                            // 10 MHz is not ok
  configureUblox();         // Configure Timepulse

// Initialisation afficheur I2C, LCD et OLED
// -----------------------------------------
  Wire.begin();
  Wire.setClock(400000L);     // vitesse transfert i2c , 400000L ou 400.000 Hz

  lcd.init();
  lcd.backlight();
  //  lcd.begin(16,4);        	  // ----- initialize the lcd for 16 chars 4 lines and turn on backlight
  lcd.begin(20,4);            // ----- initialize the lcd for 20 chars 4 lines and turn on backlight

  lcd.clear();
  lcd.setCursor(0,0);     	      // Start at position 0 at line 0 
  lcd.print(" GPSDO 40-10MHz ");
  lcd.setCursor(0,1);
  lcd.print(" Arduino NEO-8M "); 
  lcd.setCursor(0,3);
  lcd.print("  F1CJN-F4HSP-F6BUA");

/*
  #if RST_PIN >= 0                // -------------------- version 1.3 pouce --------
    oled.begin(&SH1106_128x64, I2C_ADDRESS, RST_PIN);
  #else // RST_PIN >= 0
    oled.begin(&SH1106_128x64, I2C_ADDRESS);
  #endif // RST_PIN >= 0

/*
  #if RST_PIN >= 0                // -------------------- version 0.96 pouce --------
    oled.begin(&Adafruit128x64, I2C_ADDRESS, RST_PIN);
  #else // RST_PIN >= 0
    oled.begin(&Adafruit128x64, I2C_ADDRESS);
  #endif // RST_PIN >= 0


  oled.setFont(Adafruit5x7);

  uint32_t m = micros();
  oled.clear();
  oled.set2X();
  oled.println("GPSDO 40");
  oled.set1X();
  oled.setCursor(0,3);
  oled.println(" Arduino NEO-8M");
  oled.setCursor(0,5);
  oled.println(" by F1CJN-F4HSP");
*/
  digitalWrite(LED_loop, HIGH);    // ----- turn On the LED_loop
  
  delay(4000);  						// délai maintien de l'écran de démarrage pendant 4s

  digitalWrite(LED_loop, LOW);    // ----- turn Off the LED_loop
  lcd.clear();
 // oled.clear();

}

void loop()
{
smartDelay(1000);   // 1000 nearly 1.0s loop ... ( attention, une valeur plus faible peut limiter le nombre de trames NMEA traitées !!! )
getgps(gps);       	// Get GPS parameters
read_PLL();         // Lecture et traitement de la tension de boucle
}


void calcLocator(char *dst,float lat, float lng )     // Calcule le Locator ... sur 6 digits !
{
  int lon1, lon2, lon3;
  int la1, la2, la3;
  float reste;
  
  // longitude  
  reste = lng + 180.0;
  lon1 = int(reste / 20.0);
  reste = reste - float(lon1) * 20.0;
  lon2 = int(reste / 2.0);
  reste = reste - 2.0 * float(lon2);
  lon3 = int(12.0 * reste);

  // latitude
  reste = lat + 90.0;
  la1 = int(reste / 10.0);
  reste = reste - float(la1) * 10.0;
  la2 = int(reste);
  reste = reste - float(la2);
  la3 = int(24.0 * reste);
  
  dst[0] = (char)lon1 + 65;
  dst[1] = (char)la1 + 65;
  dst[2] = (char)lon2 + 48;
  dst[3] = (char)la2 + 48;
  dst[4] = (char)lon3 + 65;     // 
  dst[5] = (char)la3 + 65;      //
}

void calcLocatorMH()            // calcul le locator MH sur 10 digits !
{
    double d;
    longitude = gps.location.lng();   // lon
    latitude = gps.location.lat();    // lat

    // convert longitude to Maidenhead

    //  STILL TO DO  keep running average of numbers representing 9th and 10th digits of Maidenhead
    //  to smooth out apparent movement of the operator as the GPS constellation changes

    d = 180.0 + longitude;
    d = 0.5 * d;
    int ii = (int) (0.1 * d);
    MH[0] = char(ii + 65);
    float rj = d - 10.0 * (float)ii;
    int j = (int) rj;
    MH[2] = char(j + 48);
    float fpd = rj - (float)j;
    float rk = 24.0 * fpd;
    int k = (int) rk;
    MH[4] = char(k + 65);
    fpd = rk - (float)(k);
    float rl = 10.0 * fpd;
    int l = (int)(rl);
    MH[6] = char(l + 48);
    fpd = rl - (float)(l);
    float rm = 24.0 * fpd;
    int mm = (int)(rm);
    MH[8] = char(mm + 65);

    //  convert latitude to Maidenhead
    d = 90.0 + latitude;
    ii = (int)(0.1 * d);
    MH[1] = char(ii + 65);
    rj = d - 10.*(float)ii;
    j = (int)rj;
    MH[3] = char(j + 48);
    fpd = rj - (float)j;
    rk = 24.0 * fpd;
    k = (int)rk;
    MH[5] = char(k + 65);
    fpd = rk - (float)(k);
    rl = 10.0 * fpd;
    l = int(rl);
    MH[7] = char(l + 48);
    fpd = rl - (float)(l);
    rm = 24.0 * fpd;
    mm = (int)(rm);
    MH[9] = char(mm + 65);
}


void getgps(TinyGPSPlus2 &gps)
{
  // Affichage --- Locator, latitude,longitude
  // -----------------------------------------
 /* 
  oled.set1X(); 
  oled.setCursor(0,0);                  // OLED 128x64 - ligne 0
  oled.print(" Loc   ");                // ----- Locator 6 digits
  oled.print (locator[0]);  
  oled.print (locator[1]); 
  oled.print (locator[2]); 
  oled.print (locator[3]); 
  oled.print (locator[4]); 
  oled.println (locator[5]); 
*/  
/*  oled.setCursor(0,0);                  // OLED 128x64 - ligne 0 
  oled.print(" Loc   ");                // ----- Locator MH 10 digits 
  oled.print (MH[0]); 
  oled.print (MH[1]); 
  oled.print (MH[2]);            
  oled.print (MH[3]); 
  oled.print (MH[4]); 
  oled.print (MH[5]); 
  oled.print (MH[6]);            
  oled.print (MH[7]); 
  oled.print (MH[8]); 
  oled.print (MH[9]);   
                        
  oled.setCursor(0,1);                  // OLED 128x64 - ligne 1 et 2 
  oled.print(" Lat   ");                // ----- Lat et Lon
  oled.print(gps.location.lat(),6);
  oled.setCursor(0,2);
  oled.print(" Long  ");  
  oled.print(gps.location.lng(),6);
*/
  
  lcd.setCursor(0,0);                   // LCD 4x16 - ligne 0 et 1
  lcd.print("Lat   ");                  // ----- Lat et Lon 
  lcd.print(gps.location.lat(),6); 
  lcd.setCursor(0,1);
  lcd.print("Lon   "); 
  lcd.print(gps.location.lng(),6);  

  lcd.setCursor(10,3);                  // LCD 4x16 - ligne 3                 
  lcd.print(locator[0]);                // ----- Locator 6 digits
  lcd.print(locator[1]);
  lcd.print(locator[2]);
  lcd.print(locator[3]);
  lcd.print(locator[4]);
  lcd.print(locator[5]);


  // Affichage --- Heure 
  // -------------------
  lcd.setCursor(0,2);                   // LCD 4x16 - ligne 2
  lcd.print("Time  ");
      if (gps.time.hour() < 10)                     // Add a leading zero
        lcd.print("0");
      lcd.print(gps.time.hour());  
  lcd.print(":");
      if (gps.time.minute() < 10)                   // Add a leading zero
        lcd.print("0");
  lcd.print(gps.time.minute());  
  lcd.print(":");
      if (gps.time.second() < 10)                   // Add a leading zero
        lcd.print("0");
  lcd.print(gps.time.second());
  lcd.print(" ");
/*
  oled.setCursor(0,4);                  // OLED 128x64 - ligne 4
  oled.print(" Heure ");
      if (gps.time.hour() < 10)                     // Add a leading zero
        oled.print("0");
      oled.print(gps.time.hour());  
  oled.print(":");
      if (gps.time.minute() < 10)                   // Add a leading zero
        oled.print("0");
  oled.print(gps.time.minute());  
  oled.print(":");
      if (gps.time.second() < 10)                   // Add a leading zero
        oled.print("0");
  oled.print(gps.time.second());
  oled.print("   ");
*/
  
  // Affichage --- Sat + Fix
  // -----------------------
  satvustotal = (gps.GPsatvus.value() + gps.GLsatvus.value());      // Calcul de la variable satvustotal
  calcLocator(locator,gps.location.lat(),gps.location.lng());       // appel fonction "calcLocator"
  calcLocatorMH();                                                  // appel fonction "calcLocatorMH" 
  
  lcd.setCursor(0,3);                     // LCD 4x16 - ligne 3 
  lcd.print("Sat       ");                // ----- Satellites used
  if (gps.satellites.value() < 10)                  
      {
      lcd.setCursor(4,3);                           // Add a leading zero
      lcd.print("0"); 
      lcd.print(gps.satellites.value()); 
      }
  else lcd.setCursor(4,3); lcd.print(gps.satellites.value());   // sat used 
  lcd.setCursor(6,3); 
  lcd.print("/"); 
  if (satvustotal < 10)                             // test valeur de la variable Satellites Vus Total ( GPS + Glonass )
      {
      lcd.setCursor(7,3);                           // Add a leading zero                           
      lcd.print("0"); 
      lcd.print(satvustotal); 
      }    
  else lcd.setCursor(7,3); lcd.print(satvustotal);  // Print variable ded satelittes vus total  

 /*
  oled.setCursor(0,5);                  // OLED 128x64 - ligne 5
  oled.print(" Sat   ");                // ----- Satellites et FIX
  if (gps.satellites.value() < 10)                          // Add a leading zero
    oled.print("0"); 
  oled.print(gps.satellites.value());                       // sat used
  oled.print("/");   
  if (satvustotal < 10)                 // test valeur de la variable Satellites Vus Total ( GPS + Glonass )                                     
    oled.print("0");                                        // Add a leading zero
  oled.print(satvustotal);                                  // Print variable des satelittes vus ( GPS + Glonass )
  oled.print("    Fix ");                                   // valeur de la variable FIX
  oled.print(gps.satfix.value());                            
*/
/*
  // Affichage --- Tension Loop
  // --------------------------
  oled.setCursor(0,7);                  // OLED 128x64 - ligne 7                
  oled.print(" Loop  ");                // ----- Tension de boucle
  oled.print(U_loop,3);                                     // affiche 3 chiffres après la virgule 
  oled.print(" v ");                                        // valeur en volts
*/
}


static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (gpsSerial.available())
      gps.encode(gpsSerial.read());
  } while (millis() - start < ms);
}

void configureUblox() 
{
  byte gpsSetSuccess = 0;
 
    buf[0]=(frequency&0x000000FF);          // Preparation little endian
    buf[1]=(frequency&0x0000FF00)>>8;
    buf[2]=(frequency&0x00FF0000)>>16;
    buf[3]=(frequency&0xFF000000)>>24;

//  Generate the configuration string for TimePulse with frequency

/* ----- UBX-CFG-TP5 parameters (not fully complete)
  0xB5, 0x62,   // header
  0x06, 0x31,   // time pulse get/set
  0x20,         // lenght 32
  0x00,         // tpIdx time pulse selection = 0 = timepulse, 1 = timepulse2  (U1 Char)
  0x00,         // reserved0 U1
  0x01, 0x00,   // reserved1 U2
  0x00, 0x32,   // antCableDelay ns
  0x00, 0x00,   // rf group delay I2
  0x00,         //
  0x00, 0x90, 0xD0, 0x03, // freqPeriod
  0x00, 0x40, 0x42, 0x0F, // freqPeriodLoc
  0x00, 0xF0, 0x49, 0x02, // pulselenRatio
  0x00, 0x60, 0xAE, 0x0A, // pulselenRatio
  0x00, 0x00, 0x00, 0x00, // user Config Delay ns
  0x00, 0x00,             // flags - page 135 u-blox 7 Receiver Description Including Protocol Specification V14.pdf
  0x00, 0x48, 0x65,       

// Exemple:       CFG-TP5 1Hz / 2 Mhz no sync 50ms cable delay
//                ----  (0xB5, 0x62, 0x06, 0x31, 0x20, 0x00, 0x00, 0x01, 0x00, 0x00, 0x32, 0x00, 0x00, 0x00,   0x01,   0x00,   0x00,   0x00,   0x80,   0x84,   0x1E,   0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x6F, 0x00, 0x00, 0x00, 0x1C, 0x1E);
// Exemple:       ----     B5    62    06    31    20    00    01    01    00    00    32    00    00    00      40      42      0F      00      40      42      0F      00    00    00    00    00    A0    86    01    00    00    00    00    00    B7    00    00    00    8B    D6
*/

  byte setTimePulse[] = {0xB5, 0x62, 0x06, 0x31, 0x20, 0x00, 0x00, 0x01, 0x00, 0x00, 0x32, 0x00, 0x00, 0x00, buf[0], buf[1], buf[2], buf[3], buf[0], buf[1], buf[2], buf[3], 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0xEF, 0x00, 0x00, 0x00, 0x91, 0x22};
  calcChecksum(&setTimePulse[2], sizeof(setTimePulse) - 4);
  delay(1000);      // 1000

  while(gpsSetSuccess < 3)
  {
   Serial.println("Setting Timepulse... "); 
   gpsSetSuccess = 0;
   while (gpsSetSuccess < 3)
   {
   Serial.println("configuration TimePulse");
   sendUBX(&setTimePulse[0], sizeof(setTimePulse));
   gpsSetSuccess += getUBX_ACK(&setTimePulse[2]);         //Passes Class ID and Message ID to the ACK Receive function
   if (gpsSetSuccess == 10) gpsStatus[7] = true;
   if (gpsSetSuccess == 5 | gpsSetSuccess == 6) gpsSetSuccess -= 4;
   }
   if (gpsSetSuccess == 3) Serial.println("Timepulse non OK");
  }
}

void calcChecksum(byte *checksumPayload, byte payloadSize) {
  byte CK_A = 0, CK_B = 0;
  for (int i = 0; i < payloadSize ;i++) {
    CK_A = CK_A + *checksumPayload;
    CK_B = CK_B + CK_A;
    checksumPayload++;
  }
  *checksumPayload = CK_A;
  checksumPayload++;
  *checksumPayload = CK_B;
}

void sendUBX(byte *UBXmsg, byte msgLength) {
  for(int i = 0; i < msgLength; i++) {
    gpsSerial.write(UBXmsg[i]);           // send UBX msg
    gpsSerial.flush();                    // Wait end of transmission
  }
    gpsSerial.println();              // send CR
    gpsSerial.flush();                // Wait end of transmission
}

byte getUBX_ACK(byte *msgID) {
  byte CK_A = 0, CK_B = 0;
  byte incoming_char;
  boolean headerReceived = false;
  unsigned long ackWait = millis();
  byte ackPacket[10] = {0xB5, 0x62, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  int i = 0;
  while (1) {
    if (gpsSerial.available()) {
      incoming_char = gpsSerial.read();
      if (incoming_char == ackPacket[i]) { i++; }
      else if (i > 2) {ackPacket[i] = incoming_char;  i++; }
    }
    if (i > 9) break;
    if ((millis() - ackWait) > 1500) {
      Serial.println("ACK Timeout");
      return 5;
    }
    if (i == 4 && ackPacket[3] == 0x00) {
      Serial.println("NAK Received");
      return 1;
    }
  }

  for (i = 2; i < 8 ;i++) {
  CK_A = CK_A + ackPacket[i];
  CK_B = CK_B + CK_A;
  }
  
  if (msgID[0] == ackPacket[6] && msgID[1] == ackPacket[7] && CK_A == ackPacket[8] && CK_B == ackPacket[9]) {
    Serial.println("Success!");
    Serial.println("ACK Received! ");
    return 10;
        }
  else {
    Serial.println("ACK Checksum Failure: ");
    delay(1000);
    return 1;
  }
}


void read_PLL()       // --- Lecture et traitement de la tension de boucle
{
  int anal_loop;      // valeur sur 12bits lue sur l'entrée A0
  val_loop = 0;       // variable pour le traitement des valeurs courantes de la tension loop 

  old_moy_val_loop = moy_val_loop; 
  moy_val_loop = 0;

  for (int i=0; i<20; i++)        // boucle d'acquisition sur une durée de 200 ms 
    {
    anal_loop = analogRead(Port_anal_loop);
    val_loop = val_loop + anal_loop ;
    delay(10);
    }

  moy_val_loop = val_loop / 20 ;

  if ( ( moy_val_loop < (old_moy_val_loop - seuil_loop)) | ( moy_val_loop > (old_moy_val_loop + seuil_loop) )  )
    {
    digitalWrite(LED_loop, LOW);    // ----- turn Off the LED_loop  
    }

  else
    {
    digitalWrite(LED_loop, HIGH);    // ----- turn On the LED_loop   
    }

  U_loop = (float)( (moy_val_loop * 5.0) / 1024.0 );    // valeur de la tension de boucle pour affichage
  
}
