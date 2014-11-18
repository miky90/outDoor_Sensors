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
#include <RF24Network.h>
#include <RF24.h>

#define MY_ADDRESS 011
#define PARENT_ADDRESS 01

// Singleton instance of the radio driver 
RF24 radio(9,10);
// Network uses that radio
RF24Network network(radio);
// Costruttore driver sensore temperatura/umidita
DHT dht;
// Costruttore driver sensore pressione
SFE_BMP180 pressure;

//#define ALTITUDE 0.0 // Altitude of SparkFun's HQ in Boulder, CO. in meters
#define STANBY_SEC 300 //Secondi tra ogni trasmissione (multiplo di 16)

//Struttura dati pacchetti rete
struct payload_Sensor_1
{
  float temp;
  float hum;
  float pres;
  float tempOfBmp;
  int battery_level; 
};

//variabili
int wakeCount; //Contatore del numero di wakeUp del watchDog (solo ogni secondi/8 volte deve inviare il segnale) 
int cycleNum;

void setup()
{ 
  Serial.begin(9600);
  Serial.println("REBOOT");
  cycleNum = (int)(STANBY_SEC/8-STANBY_SEC/95);
  wakeCount=(cycleNum-1);
  
  SPI.begin();
  // Initialize the sensor (it is important to get calibration values stored on the device).
  if (pressure.begin())
    Serial.println("BMP180 init success");
  else
  {
    Serial.println("BMP180 init fail\n\n");
    delay(50000);
    setup();
  }
  set_sleep_mode(SLEEP_MODE_PWR_SAVE);
  //inizializza sensore DHT 
  dht.setup(2, DHT::DHT22);
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_HIGH);
  network.begin(/*channel*/ 125, /*node address*/ MY_ADDRESS);  
}

void loop()
{
 if(wakeCount==(cycleNum-1))
 {
    radio.powerUp();
    // Pump the network regularly
    network.update();
    wakeCount=0;
    dht.resetTimer();
    //Controlla le condizioni e le invia ongi 75 cicli di watchDog
    char status;
    double T,P;
    float humidity, temperature;                        // You must first get a temperature measurement to perform a pressure reading.
                                                        // Start a temperature measurement:
    status = pressure.startTemperature();               // If request is successful, the number of ms to wait is returned.
    if (status != 0)                                    // If request is unsuccessful, 0 is returned.
    {
      delay(status);                                    // Wait for the measurement to complete:
      status = pressure.getTemperature(T);              // Retrieve the completed temperature measurement:
      if (status != 0)                                  // Note that the measurement is stored in the variable T.
      {                                                 // Function returns 1 if successful, 0 if failure.
        // Print out the measurement:
//        Serial.println();
//        Serial.print("temperature of BMP180: ");
//        Serial.print(T,2);
//        Serial.println(" deg C");
        status = pressure.startPressure(3);            // Start a pressure measurement:
        if (status != 0)                               // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
        {                                              // If request is successful, the number of ms to wait is returned.
                                                       // If request is unsuccessful, 0 is returned.
          delay(status);                                // Wait for the measurement to complete:
          status = pressure.getPressure(P,T);        // Function returns 1 if successful, 0 if failure.
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
    
    //recupero i valori dal sensore DHT 
    uint8_t k=3;
    do {
      Serial.println(k);
      delay(dht.getMinimumSamplingPeriod());
      humidity = dht.getHumidity();
      temperature = dht.getTemperature();
      k--;
    }
    while(dht.getStatus() == DHT::ERROR_TIMEOUT && k>0);
    //Stampo i valori appena letti
//    Serial.print("DHT Status: ");
//    Serial.println(dht.getStatusString());
//    Serial.print("tHumidity (%): ");
//    Serial.println(humidity, 1);
//    Serial.print("tTemperature (C)/t(F): ");
//    Serial.print(temperature, 1);
//    Serial.print("/ ");
//    Serial.println(dht.toFahrenheit(temperature), 1);
//    delay(100);

    //invio messaggio
    Serial.println("Sending to meteo station");
    payload_Sensor_1 payload = { temperature, humidity, P, T, NAN};
    RF24NetworkHeader header(/*to node*/ PARENT_ADDRESS);
    network.write(header,&payload,sizeof(payload));
    
    radio.powerDown();
    wdt_reset();
    myWatchdogEnable();
    sleep_mode();
 }
  else
  {
    wakeCount++;
    wdt_reset();
    myWatchdogEnable();
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
