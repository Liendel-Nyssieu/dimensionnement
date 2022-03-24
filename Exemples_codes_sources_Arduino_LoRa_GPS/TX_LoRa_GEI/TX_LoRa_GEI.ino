
/* 30 Mars 2017 - Alexandre Boyer (INSA de Toulouse)
 * TX_LoRa_GEI: émission de la station de base LoRa installée sur le toit du DGEI
 * Paramètres : Fc = 868.1 MHz, BW = 125 kHz, SF = 12, CR = 4/5, Output power = 14 dBm, explicit header,
 * preambule de 12 symboles, CRC activé, transmission en mode broadcast du message  "4IR-RT/INSAT".
 * L'émission est périodique, le message est transmis toutes les 5 secondes.
 */
#include <Wire.h>
#include <arduinoUtils.h>

// Include the SX1272 and SPI library:
#include "sx1272_INSAT.h"
#include <SPI.h>

#define freq_centrale CH_868v1
#define BW BW_125
#define CR CR_5
#define SF SF_12
#define OutPower POW_14
#define PreambLong 12
#define TX_Addr 2
#define MessageLong 12
#define MaxNbRetries 3
#define PeriodTransmission 5000 //en ms
#define WaitTxMax 2000 //en ms


uint8_t tx_address = TX_Addr;
char Message[MessageLong] = "4IR-RT/INSA";
float waitPeriod = 5000; //en ms
float TOA = 0; //en ms

// status variables
int8_t e;
boolean ConfigOK = true; //passe à false si problème d'allumage, de config de la fréquence ou de la puissance de sortie

void setup() { 

  // Open serial communications and wait for port to open:
  Serial.begin(115200);

  // Print a start message
  Serial.println(F("SX1272 module configuration in Arduino"));

  // Power ON the module
  e = sx1272.ON();
  if (e == 0)
  {
    Serial.println(F("SX1272 Module on"));
  }
  else
  {
    Serial.println(F("Problem of activation of SX1272 Module !"));
    ConfigOK = false;
  }

  // Select frequency channel
  e = sx1272.setChannel(freq_centrale);
  Serial.print(F("Frequency channel "));
  Serial.print(freq_centrale,HEX);
  if (e == 0)
  {
    Serial.println(F(" has been successfully set."));
  }
  else
  {
    Serial.println(F(" has not been set !"));
    ConfigOK = false;
  }

  // Select output power
  e = sx1272.setPower(OutPower);
  Serial.print(F("Output power "));
  Serial.print(OutPower,HEX);
  if (e == 0)
  {
    Serial.println(F(" has been successfully set."));
  }
  else
  {
    Serial.println(F(" has not been set !"));
    ConfigOK = false;
  }
  
  if (ConfigOK == true) {
    // Set header
    e = sx1272.setHeaderON();
    // Set transmission mode
    e = sx1272.setCR(CR_5);    // CR = 4/5
    e = sx1272.setSF(SF);   // SF = 12
    e = sx1272.setBW(BW_125);    // BW = 125 KHz
    // Set CRC
    e = sx1272.setCRC_ON();
    // Set the node address
    e = sx1272.setNodeAddress(tx_address);
    // Set the length of preamble
    e = sx1272.setPreambleLength(PreambLong);
    // Set the number of transmission retries
    sx1272._maxRetries = MaxNbRetries; 

    //parameters display 
    Serial.println(F("#Verification of parameters:#"));
    Serial.print(F("  Node address: "));
    Serial.println(sx1272._nodeAddress,DEC);  
    Serial.print(F("  Bandwidth: "));
    Serial.println(sx1272._bandwidth,DEC);  
    Serial.print(F("  Coding rate: "));
    Serial.println(sx1272._codingRate,DEC);
    Serial.print(F("  Spreading factor: "));
    Serial.println(sx1272._spreadingFactor,DEC);  
    Serial.print(F("  Header mode: "));
    Serial.println(sx1272._header,DEC); 
    Serial.print(F("  CRC field: "));
    Serial.println(sx1272._CRC,DEC); 
    Serial.print(F("  BW: "));
    Serial.println(sx1272._bandwidth,DEC); 
    Serial.println(F("SX1272 successfully configured !"));
  }
  else
  {
    Serial.println(F("SX1272 initialization failed !")); 
  }

  if (ConfigOK == true) {
    delay(1000);  
    Serial.println(F("  "));
    Serial.println(F("----------------------------------------"));
    Serial.println(F("Continuous transmission in broadcast mode.")); 
    Serial.println(F("----------------------------------------")); 
  } 

  TOA = sx1272.timeOnAir();
  waitPeriod = PeriodTransmission-TOA;
  Serial.print(F("  TOA (s): "));
  Serial.println(TOA,DEC); 
  if (waitPeriod < 0) {
    Serial.println(F("TOA longer than transmission period !"));
    waitPeriod = 5000;
  }
}

void loop() {

  uint8_t dest_address = BROADCAST_ADDR;  //adresse du destinataire
  int i = 0;  //compteur de paquets transmis

  if (ConfigOK == true) {
    e = sx1272.sendPacketTimeout(dest_address,Message,WaitTxMax);   

    if (e == 0) {
      Serial.print(F("\n Packet number "));
      Serial.print(i,DEC);
      Serial.println(F("\n sent."));  
      i++;
    }
    else {
      Serial.println(F("\n Trasmission problem !"));  
    }
    delay(waitPeriod); //on met une durée tampon pour émettre toutes les 5 secondes  
  }
}
