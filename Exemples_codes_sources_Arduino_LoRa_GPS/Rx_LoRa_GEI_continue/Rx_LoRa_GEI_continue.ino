/* 30 Mars 2017 - Alexandre Boyer (INSA de Toulouse)
 * Rx_LoRa_GEI_continue: programme simple deréception des trames émises par la station de base LoRa installée sur le toit du DGEI
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
#define RX_Addr 8
#define MaxNbRetries 3
#define WaitRxMax 7000 //en ms


uint8_t rx_address = RX_Addr;

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
    e = sx1272.setSF(SF_12);   // SF = 12
    e = sx1272.setBW(BW_125);    // BW = 125 KHz
    // Set CRC
    e = sx1272.setCRC_ON();
    // Set the node address
    e = sx1272.setNodeAddress(rx_address);
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
    //affichage entête
    //statut (correct = 1 or bad = 0 or non received = 2) 
    Serial.println(F("\n "));
    Serial.println(F("Module ready for reception ! "));
    Serial.println(F("Packet status ; Packet number ; Received data ; RSSI packet (dBm) ; source address"));
    Serial.println(F("\n "));
  } 
}



void loop()
{
  char StatusRXMessage;

  if (ConfigOK == true) {
    e = sx1272.receivePacketTimeout(WaitRxMax);
    //paquet reçu, correct ou non
    if (e == 0) {
      if (sx1272._reception == CORRECT_PACKET) {
        StatusRXMessage = '1';    
      }
      else {
        StatusRXMessage = '0';
      }  
    }
    //pas de réception --> paquet perdu
    else {
      StatusRXMessage = '2';  
    }  
  
    //écriture de la ligne de résultat
    Serial.print(StatusRXMessage);
    Serial.print(F(" ; "));
    Serial.print(sx1272.packet_received.packnum,DEC);
    Serial.print(F(" ; "));
    for (uint8_t i =0; i < sx1272.packet_received.length; i++) {
      Serial.print(sx1272.packet_received.data[i]);  
    }
    Serial.print(F(" ; "));
    e = sx1272.getRSSIpacket();
    Serial.print(sx1272._RSSIpacket, DEC); 
    Serial.print(F(" ; "));
    Serial.print(sx1272.packet_received.src,DEC);
  }
}

