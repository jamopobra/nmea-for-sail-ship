/*   Veleta e anemómetro RS485   
STM32:
USART_1   Tx A9   - Rx A10  Anemometro  lée vel. vento m/s
USART_2   Tx A2   - Rx A3   Veleta      Lée direcc. relativa vento 0-360º    
USART_3   Tx B10  - Rx B11  Radio VHF   Lée $CDDSC, $CDDSE
I2C_1     SCL B6  - SDA B7
Radio VHF emite sentencias NMEA183 DSC e DSE que se reciben por USART_3
========================================================================
Sentences are terminated by a <CR><LF> sequence.
Maximum sentence length, including the $ and <CR><LF> is 82 bytes.
Line_Feed       0x0A - 
Carriage_Return 0x0D - 
========================================================================
*/
#include <Wire.h>
#include "PString.h"
#include <nmea.h>
HardwareSerial Serial2(USART2); // PA3  (RX)  PA2  (TX)
HardwareSerial Serial3(USART3); // PB11 (RX)  PB10   (TX)

NMEA nmeaDecoder(ALL);
#define UInt16 uint16_t
char sentenciaCDDSE[80];
char sentenciaCDDSC[80];


float vms = 0;  // velocidade do vento en m/s
float vkn = 0;  // velocidade vento en nudos
float vkm = 0;  // velocidade vento Km/h
byte control[2];
byte test[7];
char rel_ang[2];  //para NMEA183 VWR vento relativo L / R
int grados = 0;
int ang_vento_relat = 0;  //para NMEA183 VWR vento relativo 180L / 180R

byte getChecksum(char* str)  //O checksum para as sentencias NMEA183
{
  byte cs = 0;
  for (unsigned int n = 1; n < strlen(str) - 1; n++) {
    cs ^= str[n];
  }
  return cs;
}
// A veleta e o anemómetro usan MODBUS(RS485)CRC16, nesta rutina vamos a comprobar
UInt16 Modbus_CRC16(int comproba) {  //O codigo de control para RS485 MODBUS
  UInt16 crc = 0xFFFF;

  for (int posicion = 0; posicion < comproba; posicion++) {
    crc ^= (UInt16)test[posicion];

    for (int i = 8; i != 0; i--) {
      if ((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else
        crc >>= 1;
    }
  }
  return crc;
}

void setup() {
  Serial.begin(4800);   // Recibe Anemometro
  Serial2.begin(4800);  // Recibe Veleta
  Serial3.begin(4800);  // Recibe Radio VHF (IC-M330GE) sentencias DSC e DSE
  Wire.begin();           //Envia por I2C  $CDDSC - $CDDSE - $WIMWV - $WIVWR a Wire.begin(9)
}  //no outro micro: Wire.onReceive(eventoRecepcion); rexistrar evento de recepción de datos
void loop() {

  byte Anemometro_interroga[] = { 0x1, 0x3, 0x0, 0x0, 0x0, 0x1, 0x84, 0xA };  // Anemometro
  Serial.write(Anemometro_interroga, sizeof(Anemometro_interroga));
  Serial.flush();

  byte Anemometro_buf[8];  // Vamos a léer 8 bytes do anemometro
  Serial.readBytes(Anemometro_buf, 8);
  for (int i = 0; i < 6; i++) { test[i] = Anemometro_buf[i]; }  // Os 6 primeiros bytes da trama son comprobados
  control[1] = (Modbus_CRC16(6) % 256);                         //penúltimo LOW
  control[2] = (Modbus_CRC16(6) / 256);                         //último HIGH
  if (control[1] == Anemometro_buf[6] && control[2] == Anemometro_buf[7]) {
    vms = (Anemometro_buf[3] * 255 + Anemometro_buf[4]) / 10;  // vento aparente en m/s
    vkm = vms * 3.6;                                           // kilómetros/hora
    vkn = vms * 1.94;                                          // Nudos
  }
  //  **  **  **  **  **
  byte Veleta_interroga[] = { 0x1, 0x3, 0x0, 0x0, 0x0, 0x2, 0xC4, 0xB };  //Veleta
  Serial2.write(Veleta_interroga, sizeof(Veleta_interroga));
  Serial2.flush();

  byte Veleta_buf[9];
  Serial2.readBytes(Veleta_buf, 9);                       // Leemos 9 bytes, 7 de datos e 2 de control
  for (int i = 0; i < 7; i++) { test[i] = Veleta_buf[i]; }  // Os 7 primeiros bytes da trama son comprobados
  control[1] = (Modbus_CRC16(7) % 256);                     //penúltimo LOW
  control[2] = (Modbus_CRC16(7) / 256);                     //último HIGH
  if (control[1] == Veleta_buf[7] && control[2] == Veleta_buf[8]) {
    grados = (Veleta_buf[5] * 255 + Veleta_buf[6]);  //Ángulo en º relativo á proa
    if (grados < 181) rel_ang == "R";
    if (grados > 180) rel_ang == "L";
    if (grados < 181) ang_vento_relat == grados;
    if (grados > 180) ang_vento_relat == (360 - grados);
  }
  MWV();  //Unha vez leidos anemometro e veleta pasamos a unha rutina que fabrique
  VWR();  //sentenzas NMEA183 e enviaas por I2C ó outro micro
  //$CDDSC e $CDDSE dende a radio VHF
  if (Serial3.available()) {
    if (nmeaDecoder.decode(Serial3.read())) {
      char* title = nmeaDecoder.term(0);
      if (strcmp(title, "CDDSE") == 0) {
        char* sentenciaCDDSE = nmeaDecoder.sentence();
        DSE();
      }
      if (strcmp(title, "CDDSC") == 0) {
        char* sentenciaCDDSC = nmeaDecoder.sentence();
        DSC();
      }
    }
  }
}

// E fin do <void loop>, xa enviou as duas sentenzas do vento e as duas da radio. Fin do programa.
void DSE() {

  Wire.beginTransmission(9);  // transmite ao receptor #9
  for (byte i = 0; i < sizeof(sentenciaCDDSE) - 1; i++) { Wire.write(sentenciaCDDSE[i]); }
  Wire.write(0x0A);
  Wire.write(0x0D);
  Wire.endTransmission();  // stop transmision
  delay(5);
}
void DSC() {

  Wire.beginTransmission(9);  // transmit to device #9
  for (byte i = 0; i < sizeof(sentenciaCDDSC) - 1; i++) { Wire.write(sentenciaCDDSC[i]); }
  Wire.write(0x0A);
  Wire.write(0x0D);
  Wire.endTransmission();  // stop transmitting
  delay(5);
}
void MWV() {              //Fabricamos sentecia $XXMWV de NMEA183
  char SentenciaMWV[80];  //reservamos un mínimo de caracteres para esta cadea $XXMWV
  byte cs;                //para o checksum
  //emsamblamos a sentencia
  PString str(SentenciaMWV, sizeof(SentenciaMWV));
  str.print("$WIMWV,");            //MWV, velocidade e angulo do vento
  str.print(grados);               // angulo 0 - 360º
  str.print(",R,");                //relativo
  str.print(vms, 1);               // velocidade vento
  str.print(",M,A*");              // M=m/s A=Status valido
  cs = getChecksum(SentenciaMWV);  //calcula o checksum
  //caracteristica de arduino: imprime 0x007 como 7, 0x02B como 2B, necesita un arreglo
  if (cs < 0x10) str.print('0');
  str.print(cs, HEX);  // terminase de ensamblar toda a sentencia e enviase ao porto
  str.print(0x0A);
  str.print(0x0D);            //Line_Feed       0x0A - Carriage_Return 0x0D -
  Wire.beginTransmission(9);  // transmit to device #9
  for (byte i = 0; i < sizeof(SentenciaMWV) - 1; i++) { Wire.write(SentenciaMWV[i]); }
  Wire.endTransmission();  // stop transmitting
  delay(5);
}
void VWR() {              //Fabricamos sentecia $XXVWR de NMEA183
  char SentenciaVWR[80];  //reservamos un mínimo de caracteres para esta cadea $XXVWR
  byte cs;                //para o checksum
  //emsamblamos a sentencia
  PString str(SentenciaVWR, sizeof(SentenciaVWR));
  str.print("$WIVWR,");        //VWR, velocidade e angulo do vento
  str.print(ang_vento_relat);  // angulo 0 - 180º
  str.print(",");
  str.print(rel_ang);  // L ou R
  str.print(",");
  str.print(vkn, 1);  //velocidade vento en knots, 1 decimal
  str.print(",N,");
  str.print(vms, 1);  //velocidade vento metros/segundo 1 decimales
  str.print(",M,");
  str.print(vkm, 1);  //velocidade vento Km/hora, 1 decimal
  str.print(",K*");
  cs = getChecksum(SentenciaVWR);  // calcula o checksum
  //caracteristica de arduino: imprime 0x002 como 2, 0x03E como 3E, necesita un arreglo
  if (cs < 0x10) str.print('0');  //  este e o arreglo
  str.print(cs, HEX);             //  terminase de ensamblar toda a sentencia e enviase ao porto
  str.print(0x0A);
  str.print(0x0D);            //Line_Feed       0x0A - Carriage_Return 0x0D -
  Wire.beginTransmission(9);  // transmite ao destino #9
  for (byte i = 0; i < sizeof(SentenciaVWR) - 1; i++) { Wire.write(SentenciaVWR[i]); }
  Wire.endTransmission();  // fin da transmisión.
  delay(5);
}
/*
if(ais_sentencia.indexOf("3pr@W") > 0){  //Si encuentra el carácter, entraremos en el if
 }
*/
//     Baud rate: 4800 bauds.    Data bits: 8.    Parity: none.    Stop bit: none.
/*=== MWV - Wind Speed and Angle ===
 *         1   2 3   4 5
 *         |   | |   | |
 *  $--MWV,x.x,a,x.x,a*hh<CR><LF>
 * ------------------------------------------------------------------------------
 *
 * Field Number:
 * 1. Wind Angle, 0 to 360 degrees
 * 2. Reference, R = Relative, T = True
 * 3. Wind Speed
 * 4. Wind Speed Units, K/M/N
 * 5. Status, A = Data Valid
 * 6. Checksum
 */
/*
$--VWR,x.x,a,x.x,N,x.x,M,x.x,K*hh<CR><LF>
x.x Measured Win  Angle relative to the vessel, 0º to 180º, left / right L/R of vessel heading
a L /R
Measured wind speed Knots (N)
Measured wind speed meters/second (M)
Measured wind Speed Km/h. (K)
* hh Código control
$XXVWR,90.0,R,21.6,N11.11,M,40.0,K*7D
vento 90º estribor 21.6 nudos 11.11 m/s 40.0 km/h
*/

/*VENTO REAL A PARTIRES DE VENTO APARENTE
A Velocidade vento aparente en knots con 1 decimal, 
V SOG barco en knots con un decimal
ß Ángulo vento aparente respecto á cruxía (escribir letra ß en windows ALT 225)
W vento real velocidade en knots
alfa Ángulo vento real respecto a cruxia

W = raiz cadrada de (A2 + V2 - 2AV * cos ß), esta é a velocidade do vento real

alfa = arcos((A * cosß - V)/W) este é o ángulo do vento real respecto á cruxía

*/