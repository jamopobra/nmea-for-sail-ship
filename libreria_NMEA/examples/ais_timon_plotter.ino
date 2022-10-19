/*   Veleta e anemómetro RS485   
STM32:
USART_    Tx A9     Rx A10    AIS
USART_2   Tx2 A2    Rx2 A3    Timon ST2000+  
USART_3   Tx3 B10   Rx3 B11   Plotter Garmin GPSMAP 700S
I2C_1     SCL B6    SDA B7    Wire I2C
I2C_2     
Radio VHF emite sentencias NMEA183 DSC e DSE que se reciben por USART_3
*/

#include "Wire.h"
#include <nmea.h>
HardwareSerial Serial2(USART2);  // PA3  (RX)  PA2  (TX)
HardwareSerial Serial3(USART3);  // PB11 (RX)  PB10 (TX)
String SentenciaAIVDM;
String SentenciaGPAPB;
String SentenciaGPBWC;
String SentenciaGPRMB;
String SentenciaGPXTE;
String SentenciaSDVHW;
const int conmutadorPin = PB9;
int estadoConmutador = 0;
#define I2c_direcc 0x09
NMEA nmeaDecoder(ALL);

void setup() {
  pinMode(conmutadorPin, INPUT);  //PB9 declarase como input
  Serial.begin(4800);     //  AISS RECEIVER
  Serial2.begin(4800);    //  TIMON
  Serial3.begin(4800);    //  PLOTTER
  Wire.begin(I2c_direcc); //recibe por I2C  $CDDSC - $CDDSE - $WIMWV - $WIVWR a Wire.begin()
  Wire.onReceive(RECEV);  // rexistro evento
}  //neste micro: Wire.onReceive(eventoRecepcion); rexistrar evento de recepción de datos

void loop() {
  estadoConmutador = digitalRead(conmutadorPin);
  if (Serial.available()) {
    if (nmeaDecoder.decode(Serial.read())) {
      char* title = nmeaDecoder.term(0);
      if (strcmp(title, "AIVDM") == 0) {
        SentenciaAIVDM = nmeaDecoder.sentence();
        VDM();
        if (estadoConmutador == HIGH) {
          VDM2();
        }
      }
    }
  }
  if (Serial3.available()) {
    if (nmeaDecoder.decode(Serial3.read())) {
      char* title = nmeaDecoder.term(0);
      if (strcmp(title, "GPAPB") == 0) {
        SentenciaGPAPB = nmeaDecoder.sentence();
        APB();
      }
      if (strcmp(title, "GPBWC") == 0) {
        SentenciaGPBWC = nmeaDecoder.sentence();
        BWC();
      }
      if (strcmp(title, "GPRMB") == 0) {
        SentenciaGPRMB = nmeaDecoder.sentence();
        RMB();
      }
      if (strcmp(title, "GPXTE") == 0) {
        SentenciaGPXTE = nmeaDecoder.sentence();
        XTE();
      }
      if (strcmp(title, "SDVHW") == 0) {
        SentenciaSDVHW = nmeaDecoder.sentence();
        VHW();
      }
    }
  }
}

void VDM() {                                  //Serial 3 escribe no plotter
  if (SentenciaAIVDM.indexOf("3pr@W") < 0) {  // <0 ou -1 o buscado non está no string, >0  encontrou o buscado
    Serial3.println(SentenciaAIVDM);          //Serial.println agrega \r e \n . Retorno de carro salto de liña.
  }
}
void VDM2() {                                 //Serial 3 escribe no plotter
  if (SentenciaAIVDM.indexOf("3pr@W") > 0) {  // <0 ou -1 o buscado non está no string, >0  encontrou o buscado
    Serial3.println(SentenciaAIVDM);          //Serial.println agrega \r e \n . Retorno de carro salto de liña.
  }
}
void APB() {  //Serial 2, escribe no timon
  Serial2.println(SentenciaGPAPB);
}
void BWC() {  //Serial 2, escribe no timon
  Serial2.println(SentenciaGPBWC);
}
void RMB() {  //Serial 2, escribe no timon
  Serial2.println(SentenciaGPRMB);
}
void XTE() {  //Serial 2, escribe no timon
  Serial2.println(SentenciaGPXTE);
}
void VHW() {  //Serial 2, escribe no timon
  Serial2.println(SentenciaSDVHW);
}
void RECEV(int k) {  //Vamos a ler de Wire $CDDSC - $CDDSE - $WIMWV - $WIVWR
  char recibe[80];
  String Sentenciawire;
  int j = 0;
  while (1 < Wire.available()) {  // loop through all but the last
    recibe[j] = Wire.read();      // receive byte as a character
    j++;
  }
  Sentenciawire = String(recibe);
  if (Sentenciawire.indexOf("WIVWR") > 0) {  // <0 ou -1 o buscado non está no string, >0  encontrou o buscado
    Serial2.println(Sentenciawire);
  } else {
    Serial3.println(Sentenciawire);
  }
}