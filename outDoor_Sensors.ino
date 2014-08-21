/*
*  File:     OutDoor_Sensors.ino
*  Author:   Marco Michielotto
*  Date:     01/06/2014
*  Hardware: Arduino Pro Mini(3.3V, 8Mhz)w/ ATmega328,
*            DHT22 on pin D2 whith pullUp 10k resistor,
*            BMP180 I2C connection  A4 (SDA) and A5 (SCL),
*            NRF2401L+ on SPI connection (see below)
*/

//Connect the nRF24L01 to Arduino like this. Use these same connections for Teensy 3.1.
// \code
//                 Arduino      NRF2401L+
//                 3V3 or 5V----VCC   (3.3v) or (3.3V to 7V in only for Sparkfun WRL-00691) 
//             pin D8-----------CE    (chip enable in) -------------- N.B.: Library default but in code below is set D9 pin !
//          SS pin D10----------CSN   (chip select in)
//         SCK pin D13----------SCK   (SPI clock in)
//        MOSI pin D11----------SDI   (SPI Data in)
//        MISO pin D12----------SDO   (SPI data out)
//                              IRQ   (Interrupt output, not connected)
//                 GND----------GND   (ground in)
// \endcode

#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <SFE_BMP180.h>
#include <Wire.h>
#include "DHT.h"
#include <SPI.h>
#include <RH_NRF24.h>

// Singleton instance of the radio driver 
RH_NRF24 nrf24(9);
// Costruttore driver sensore temperatura/umidita
DHT dht;
// Costruttore driver sensore pressione
SFE_BMP180 pressure;

//#define ALTITUDE 0.0 // Altitude of SparkFun's HQ in Boulder, CO. in meters
#define STANBY_SEC 300 //Secondi tra ogni trasmissione (multiplo di 16)

int wakeCount; //Contatore del numero di wakeUp del watchDog (solo ogni secondi/8 volte deve inviare il segnale) 
int cycleNum;
void setup()
{ 
  Serial.begin(9600);
  Serial.println("REBOOT");
  cycleNum = (int)(STANBY_SEC/8-STANBY_SEC/95);
  wakeCount=(cycleNum-1);
  
  // Initialize the sensor (it is important to get calibration values stored on the device).
  if (pressure.begin())
    Serial.println("BMP180 init success");
  else
  {
    // Oops, something went wrong, this is usually a connection problem,
    // see the comments at the top of this sketch for the proper connections.
    Serial.println("BMP180 init fail\n\n");
    delay(50000);
    setup();
  }
  
  set_sleep_mode(SLEEP_MODE_PWR_SAVE);
  //inizializza sensore DHT 
  dht.setup(2); // data pin 2
  if (!nrf24.init())
    Serial.println("init failed");
  if (!nrf24.setChannel(10))
    Serial.println("setChannel failed");
  if (!nrf24.setRF(RH_NRF24::DataRate250kbps, RH_NRF24::TransmitPower0dBm))
    Serial.println("setRF failed");    
}

void loop()
{
  
 if(wakeCount==(cycleNum-1))
 {
   wakeCount=0;
    //Controlla le condizioni e le invia ongi 75 cicli di watchDog
    char status;
    double T,P;
    float humidity, temperature;
    // You must first get a temperature measurement to perform a pressure reading.
    
    // Start a temperature measurement:
    // If request is successful, the number of ms to wait is returned.
    // If request is unsuccessful, 0 is returned.
    status = pressure.startTemperature();
    if (status != 0)
    {
      // Wait for the measurement to complete:
      delay(status);
  
      // Retrieve the completed temperature measurement:
      // Note that the measurement is stored in the variable T.
      // Function returns 1 if successful, 0 if failure.
  
      status = pressure.getTemperature(T);
      if (status != 0)
      {
        // Print out the measurement:
//        Serial.println();
//        Serial.print("temperature of BMP180: ");
//        Serial.print(T,2);
//        Serial.println(" deg C");
//        
        // Start a pressure measurement:
        // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
        // If request is successful, the number of ms to wait is returned.
        // If request is unsuccessful, 0 is returned.
        status = pressure.startPressure(3);
        if (status != 0)
        {
          // Wait for the measurement to complete:
          delay(status);
  
          // Retrieve the completed pressure measurement:
          // Note that the measurement is stored in the variable P.
          // Note also that the function requires the previous temperature measurement (T).
          // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
          // Function returns 1 if successful, 0 if failure.
  
          status = pressure.getPressure(P,T);
          if (status != 0)
          {
            // Print out the measurement:
//            Serial.print("absolute pressure: ");
//            Serial.print(P,2);
//            Serial.println(" mb");
          }
          else Serial.println("error retrieving pressure measurement\n");
        }
        else Serial.println("error starting pressure measurement\n");
      }
      else Serial.println("error retrieving temperature measurement\n");
    }
    else Serial.println("error starting temperature measurement\n");
    
    //delay(dht.getMinimumSamplingPeriod());
    
    //recupero i valori dal sensore DHT 
    humidity = dht.getHumidity();
    temperature = dht.getTemperature();
  
    //Stampo i valori appena letti
//    Serial.print("DHT Status: ");
//    Serial.println(dht.getStatusString());
//    Serial.print("tHumidity (%): ");
//    Serial.println(humidity, 1);
//    Serial.print("tTemperature (C)/t(F): ");
//    Serial.print(temperature, 1);
//    Serial.print("/ ");
//    Serial.println(dht.toFahrenheit(temperature), 1);
    //delay(100);
    
    //creo le stringhe con i dati appena rilevati
    char pressione[7]; 
    dtostrf(P,4,2,pressione);
    char umidita[6];
    dtostrf(humidity,3,2,umidita);
    char temperatura[6];
    dtostrf(temperature,3,2,temperatura);
    
    //creo messaggio da inviare
    uint8_t data[20];
    data[0]='p';
    for(int i=1;i<8;i++) 
      data[i]=pressione[(i-1)];
    data[8]='u';
    for(int i=9;i<14;i++) 
      data[i]=umidita[(i-9)];
    data[14]='t';
    for(int i=15;i<20;i++) 
      data[i]=temperatura[(i-15)];
    data[20]='\0';
   
    //invio messaggio
//    Serial.println("Sending to nrf24_server");
//    Serial.println((char*)data);
    // Send a message to nrf24_server
    nrf24.send(data, sizeof(data));
    nrf24.waitPacketSent();
    delay(100);
    //power down
    nrf24.setModeIdle();
  
    //pausa 
    //delay(299900);
    wdt_reset();
    myWatchdogEnable();
    //Serial.println("...going to sleep.");
    //delay(100);
    sleep_mode();
 }
 else
 {
   wakeCount++;
   //Serial.print(wakeCount);
   //delay(100);
   wdt_reset();
   myWatchdogEnable();
   //Serial.println("...going to sleep.");
   //delay(100);
   sleep_mode();
 }
}


/***************************************************
 *  Name:        ISR(WDT_vect)
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Watchdog Interrupt Service. This
 *               is executed when watchdog timed out.
 *
 ***************************************************/
ISR(WDT_vect)
{
  cli();
  wdt_disable();
  //Serial.println("wakeup!");
  sei();
}

void myWatchdogEnable() {  // turn on watchdog timer; interrupt mode every 2.0s
  cli();
  MCUSR = 0;
  WDTCSR |= B00011000;
  // WDTCSR = B01000111; // 2 second WDT 
  //WDTCSR = B01100000;    // 4 second WDT 
  WDTCSR = B01100001;  // 8 second WDT 
  sei();
}
