#include <SPI.h>

#include <SoftwareSerial.h> // Soft Serial to free 0,1 pins
SoftwareSerial BTSerial(4,5); // Soft Serial on 4,5 pins // rx, tx

#define LED 13
#define adcChipSelectPin 9 // SPI Slave Select pin

String inValue;
String outPut;
int mainCounter;
int subCounter;
int startFlag;


//#define UpperThreshold 518 // ADC 10 bits
//#define LowerThreshold 495 // ADC 10 bits

#define UpperThreshold 2080 // ADC 12 bits
#define LowerThreshold 1950 // ADC 12 bits

unsigned long first_pulse_time;
unsigned long second_pulse_time;
bool firstPeak = false;
bool getValue = true;
unsigned long RR_Interval;
float BPM;
byte val;


void setup() {
  
  // initialize the serial communication:
  Serial.begin(115200);
  BTSerial.begin(115200);
 
  
  pinMode(LED, OUTPUT);
  outPut="";
  pinMode(10, INPUT); // Setup for leads off detection LO +
  pinMode(11, INPUT); // Setup for leads off detection LO -
  startFlag=0;

  // SPI config
  pinMode(adcChipSelectPin, OUTPUT); 
  digitalWrite(adcChipSelectPin, HIGH);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV8);

}

void loop() {

  // MCP3208 code start
  byte adcPrimaryRegister = 0b00000110;
  byte adcPrimaryRegisterMask = 0b00000111;
  byte adcPrimaryByteMask = 0b00001111;
  byte adcPrimaryConfig = adcPrimaryRegister & adcPrimaryRegisterMask;
  byte adcSecondaryConfig = 0 << 6;
  
  noInterrupts();
  digitalWrite(adcChipSelectPin,LOW);
  SPI.transfer(adcPrimaryConfig);
  byte adcPrimaryByte = SPI.transfer(adcSecondaryConfig);  
  byte adcSecondaryByte = SPI.transfer(0x00);
  digitalWrite(adcChipSelectPin,HIGH);
  
  interrupts();
  adcPrimaryByte &= adcPrimaryByteMask;
  int digitalValue = (adcPrimaryByte << 8) | adcSecondaryByte;
  // MCP3208 code end


//Serial.println(analogRead(A0));

if(digitalValue >= UpperThreshold && getValue == true){

    if(firstPeak == false){
      first_pulse_time = millis();
//      Serial.println(first_pulse_time);
      firstPeak = true;
    }
    else {
      second_pulse_time = millis();
//      Serial.println(second_pulse_time);
      RR_Interval = second_pulse_time - first_pulse_time;
      first_pulse_time = second_pulse_time;
      
    }
    getValue = false;
}

if(digitalValue <= LowerThreshold){
  getValue = true;
}  

if(RR_Interval) {
  BPM = (1.0 / RR_Interval) * 60.0 * 1000;
} else {
  BPM = 0.0;
}

  
  if (BTSerial.available() > 0) {
    byte a = BTSerial.read();
    if (a==(byte) 's'){
      subCounter=0;
      mainCounter=0;
      startFlag=1;
    }
  }


if((digitalRead(10) == 1)||(digitalRead(11) == 1)){
    Serial.println('!');
}
else{
  
  if (startFlag==1 && mainCounter <= 1301){
    
    // read the value of analog input
    inValue = digitalValue;
    outPut = outPut + inValue;
    mainCounter++;
    subCounter++;
    
      if (subCounter == 20 && mainCounter <= 1300){
        
        BTSerial.println( outPut );
        Serial.println( outPut );
        subCounter=0;
        outPut="";
        digitalWrite(LED, HIGH);
        
      }else if (mainCounter == 1301) {
      outPut="";
      outPut = BPM;
      BTSerial.println( outPut );
      Serial.println( outPut );
      startFlag = 0;
    } else {
        outPut = outPut + " ";
    }
  }
}
  
  delay(6); // some delay to keep serial data from saturating
  digitalWrite(LED, LOW);
  
}
