#include <Audio.h>
#include "fluteModel.h"
#include <Wire.h>
#include "Adafruit_MPR121.h"

Adafruit_MPR121 cap = Adafruit_MPR121();
uint16_t previousPosition = 0;
uint16_t currentPosition = 0;

fluteModel myDsp;
AudioOutputI2S out;
AudioControlSGTL5000 audioShield;
AudioConnection patchCord0(myDsp,0,out,0);
AudioConnection patchCord1(myDsp,0,out,1);

const float ADC_mV = 4.8828125;       // convesion multiplier from Arduino ADC value to voltage in mV
const float SensorOffset = 200.0;     // in mV taken from datasheet
const float sensitivity = 4.413;      // in mV/mmH2O taken from datasheet
const float mmh2O_cmH2O = 10;         // divide by this figure to convert mmH2O to cmH2O
const float mmh2O_kpa = 0.00981;      // convesion multiplier from mmH2O to kPa
float startUpPressure;

void setup() 
  {
    myDsp.setParamValue("mouthPosition", 0.5);
    AudioMemory(6);
    audioShield.enable();
    audioShield.volume(0.5);
    myDsp.setParamValue("pressure", 0.72);
    startUpPressure = analogRead(A0);
    Serial.print("Startup pressure : ");
    Serial.println(startUpPressure);
    if (!cap.begin(0x5A)) 
      {
        Serial.println("MPR121 not found, check wiring?");
        while (1);
      }
    Serial.println("MPR121 found!");
  }

  float calculateGain()
    {
      float res;
      float sensorValue = analogRead(A0)- startUpPressure;
      if(sensorValue<5) return 0;
      res = sensorValue*0.0009765625*0.35+0.7;
      if (res>0.89) return 0.89;
      else return res; // 1/1024 (gain max)
    }

  float calculateFrequency(String fP)
    {
      float res;
      if(fP=="01111111111") res=130.81;
      else if(fP=="01111111110") res=277.18;
      else if(fP=="01111111100") res=293.66;
      else if(fP=="01111111000") res=311.13;
      else if(fP=="01111110000") res=329.63;
      else if(fP=="01111100000") res=349.23;
      else if(fP=="01111010000" || fP=="01111011000" || fP=="01111011100") res=369.99;
      else if(fP=="01111000000") res=392.00;
      else if(fP=="01110100000" || fP=="01110110000") res=415.30;
      else if(fP=="01110000000") res=440.00;
      else if(fP=="01101000000" || fP=="01101100000" || fP=="01101101000") res=466.16;
      else if(fP=="01100000000") res=493.88;
      else if(fP=="01010000000") res=523.25;
      else res=450.00;
      return res;
    }

 float calculateFrequency2(String fP)
  {
    float res;
    if (fP=="10111111111") res = 1.56 ; //do   261.63
    else if (fP=="10111111110") res = 1.45;//do#  277.18
    else if (fP=="10111111100") res = 1.34;//ré 293.66
    else if (fP=="10111111000") res = 1.23;//mi b 311.13
    else if (fP=="10111110000") res = 1.16;//mi 329.63
    else if (fP=="10111100000" || fP=="10111101111" || fP=="10111101100")res = 1.065;//fa 349.23
    else if (fP=="10111010000" || fP=="10111011000" || fP=="10111011100")res = 0.99;//fa# 369.99
    else if (fP=="10111000000") res = 0.9;//sol 392.00
    else if (fP=="10110100000" || fP=="10110110000") res= 0.83;//sol#  415.30
    else if (fP=="10110000000")res = 0.77;//la 440.00
    else if (fP=="10101000000" || fP=="10101100000" || fP=="10101101100")res = 0.7;//si b 466.16
    else if (fP=="10100000000")res = 0.65;//si 493.88
    else if (fP=="10010000000" || fP=="01111111111")res = 0.6;//do2 523.25
    else if (fP=="00110000000" || fP=="01111111110" || fP=="00111111110") res = 0.545;//do#2 554.37
    else if (fP=="01000000000" || fP=="01111111100" || fP=="00111111100")res = 0.49;//ré2 587.33
    else if (fP=="01111111000" || fP=="00111111000" || fP=="00011111000")res = 0.445;//mi b2 622.25
    else if (fP=="01111110000" || fP=="00111110000" || fP=="00011110000")res= 0.397;//mi2 659.25
    else if (fP=="01111100000" || fP=="00111100000" || fP=="00111101100")res= 0.355;//fa2 698.46
    else if (fP=="01111010000" || fP=="01111011100" || fP =="00111010000")res= 0.32;//fa# 2 739.99
    else if (fP=="01111000000")res= 0.28; // sol2 783.99
    else if (fP=="01110100000" || fP=="01110100000" || fP=="01110110000")res= 0.245;//sol# 2 830.61
    else if (fP=="01110000000")res= 0.21;//la 2 880.00
    else if (fP=="01101000000" || fP=="01101100000" || fP=="01101101100")res=0.18;//si b2 932.33
    else if (fP=="01100000000")res= 0.148;//si 2 987.77
    else if (fP=="01010000000" || fP=="11111111111")res= 0.12;// do3 1046.50
//    else if (fP=="11111111110")res=1108.73;//do#3
//    else if (fP=="11111111100")res=1174.66;//ré3
//    else if (fP=="11111111000")res=1244.51;//mi b3
//    else if (fP=="11111110000")res=1318.51;//mi 3
//    else if (fP=="11111100000" || fP=="11111101100" || fP=="11111101111")res=1396.91;//fa 3
//    else if (fP=="11111010000" || fP=="11111011100")res=1479.98;//fa#3
//    else if (fP=="11111000000")res=1567.98;//sol 3
//    else if (fP=="11110100000" || fP=="11110110000")res=1661.22;//sol #3
//    else if (fP=="11110000000")res=1760.00;//la3
//    else if (fP=="11101000000"|| fP=="11101100000" || fP=="11101101100")res=1864.66;//si b3
//    else if (fP=="11100000000")res=1975.53;//si 3

    else res=1;
    return res;
  }

String getFingerPosition()
  {
    String finalPosition="";
    currentPosition = cap.touched();
    if(previousPosition==currentPosition) return "same";
    
    for (int i=0; i<11; i++)
      {
        if ((currentPosition & (1 << i))) finalPosition+="1";
        else finalPosition+="0";
      }
      
    previousPosition = currentPosition;
    return finalPosition;
  }
  
void loop() 
  {
    String fingerPosition = getFingerPosition();
    if(fingerPosition!="same")
      {
        float freq;
        Serial.print("Position: ");
        Serial.print(fingerPosition);
        Serial.print("\t");
        freq = calculateFrequency2(fingerPosition);
        Serial.print("Frequency: ");
        Serial.print(freq);
        Serial.print("\t");
        myDsp.setParamValue("tubeLength", freq);
      }

    float finalGain = calculateGain();
    Serial.print("Gain: ");
    Serial.println(finalGain);
    myDsp.setParamValue("pressure", finalGain);
//    Serial.println(myDsp.getParamValue("pressure"));
//    Serial.println(myDsp.getParamValue("frequency"));
    delay(10);
  }
