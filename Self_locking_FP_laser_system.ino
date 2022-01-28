#include <RunningMedian.h>
#include <MovingAverageFilter.h>
#include <stdint.h>
#include <math.h>

// fast read/write variables
int Areg = 0;
int NAreg = 0;
int Breg = 0;
int NBreg = 0;
int Creg = 0;
int NCreg = 0;
int Dreg = 0;
int NDreg = 0;

unsigned long t_measure_0 = 0;
unsigned long t_measure_1 = 0;

// ADC set up
int ADCpins[16]; //ADC digital pin locations
int ADCout = 0; //output of ADC (convert from signed binary into decimals integer)

//DAC set up
int DACpins[16]; //DAC digital pin locations
int CLR = 2;     //Clear switch for the LTC1821
int WR = 3;      //Write switch for the LTC1821
int LD = 4;      //Load switch for the LTC1821
//int wastedspace = 0;

//int Output;
int D13 = 13;
int cycleTimePin = 12;


////Valiables
int D8 = 8;
int D9 = 9;
int sample1;
int sample2;
int sample1new = 0;
int sample2new = 0;
int output = 0;
int a = 1;
int out = 0;
int out0 = 0;
int out00 = 0;
int out000 = 0;
int v1 = 0;
int v2 = 0;
int out_1 = 0;
int out_2 = 0;
int out_3 = 0;

  int N = 500; // 500; // number of step
  int N_samp = 200;
  int v_step = 32768/N; //32768/(250);// step of voltage
  int v_init = 0; // voltage which input to LDC
  int outAvg = 0;
  int outMax = -5000;
  int maxPos = 0;
  int maxPosTotal = 0;
  int outMin = 5000;
  int minPos = 0;
  int minPosTotal = 0;
  int avgPos = 0;
  int avgPos1 = 0;
  int ManScanTrig = 0;
  float error = 0;
  float error1 = 0;
  float accumulator = 0;
  float diff = 0;
  float Pterm = 0;
  float Iterm = 0;
  float Dterm = 0;
  float kp = 0;
  float ki = 0;
  float kd = 0;
  float PIcontrol = 0;
  int count_1 = 0;
  bool next = true;
  bool next_2 = true;
  bool nextSweep = true;
  int thresholdCount = 0;
  int Peak = 0;
  int shiftedOutput = 0;
  int outShifted = 0;
  int gotoLock = 0;
  int threshold_1;
  int v_initSum = 0;
  int v_initAvg = 0;
  int v_initAvg_1 = 0;
  int v_initAvg_2 = 0;
  int v_initLock = 0;
  int v_initLock1 = 0;
  int relockCount = 0;
  int integral = 0;
  int proportional = 0;
  int differential = 0;
  int startLockCount = 0;
  int lockCount = 0;
  int setpoint = 0;
  int max_1 = 0;
  int min_1 = 0;
  int x = 22000 ;
  float v_initLockTotal = 0;
  float v_initLockPrev = 0;
  float v_initLockAvg = 0;
  float errorTotal = 0;
  float errorPrev = 0;
  float errorAvg = 0;
  float theCurrent = 0;
  float minCurrent = 0;
  float maxCurrent = 0;
  float diffCurrent = 0;
  float sizeCurrent = 0;
  

MovingAverageFilter movingAverageFilter(3);
RunningMedian samples = RunningMedian(15);
RunningMedian Current = RunningMedian(10);

int kpPin = A4;
int kiPin = A5;
int kdPin = A6;
float calcIntegral(){
  integral = analogRead(kiPin);
  integral = integral * 0.05;
  return integral;
}
float calcProportional(){
  proportional = analogRead(kpPin);
  proportional = proportional * 0.001;
  return proportional;
}

float calcDifferential(){
  differential = analogRead(kdPin);
  differential = differential * 0.0005;
  return differential;
}
//====================== START UP LOOP: ======================================
void setup() {
  Serial.begin(9600); //Allows the user to view values on the PC
  pinMode(D8, OUTPUT);
  pinMode(D13, OUTPUT);
  pinMode(D9, OUTPUT);
  pinMode(cycleTimePin, OUTPUT);
  digitalWrite(cycleTimePin,LOW);

// ADC set up
for (int i=0; i <= 15; i++){
      pinMode(22+(i*2), INPUT);
      ADCpins[i] = 22+(i*2); //This line stores the pin numbers used by the ADC for later access
   }

// DAC set up
//These 7 lines define outputs that will control the LTC1821 and clears its registry
pinMode(WR, OUTPUT); //Declares the pins I will use to control the LTC1821's write
pinMode(LD, OUTPUT);
pinMode(CLR, OUTPUT);

digitalWrite(CLR, 1);
digitalWrite(CLR, 0);//Clears the LTC1821's regester and outputs setting them to zero
digitalWrite(WR, 1); //Tells the LTC1821 (DAC) not to load to the register.
digitalWrite(LD, 0); //Tells the LTC1821 (DAC) not to output from the register.
digitalWrite(CLR, 1); //Stops clearing.

analogReadResolution(12);
analogWriteResolution(12);

//This for loop declares pins 22,24,...,52 as outputs  
  for (int i=0; i <= 15; i++){     //(int i=11; i <= 27; i++){
    pinMode(53-(i*2), OUTPUT);     // pinMode(i*2, OUTPUT); //DACpins[i-11] = i*2
    DACpins[i] = 53-(i*2); //This line stores the pin numbers used by the DAC for later access 
  }
}

//========================= MAIN LOOP: ===========================================
void loop() {
  //--------------------Sweep Loop--------------------//
  while(count_1 < 30){
    v_init = 15000;
    next = true;
    while(next == true){
      out0 = calculateOutput();
      out_1 = ((out0/4) + 2048); //+2048
      writeDAC16fast(v_init);
      analogWrite(DAC1, out_1);
             
      if(v_init > 15000 + 500 && v_init < 32768 - 500){
        outAvg = outAvg + out_1;
     
        //Maximum position//
        if(out_1 > outMax){
          outMax = out_1;
          maxPos = v_init;
        }
        
        //Minimum position//
        if(out_1 < outMin){
          outMin = out_1;
          minPos = v_init;
        }
      }
      if(v_init < 32768 - x){
        v_init = v_init + 10;
      }
      else{
        next = false;
        v_init = 0;
      }
    }
    maxPosTotal = maxPosTotal + maxPos;
    minPosTotal = minPosTotal + minPos;
    count_1++;
  }
  if(count_1 == 30){
    maxPosTotal = maxPosTotal / 30;
    minPosTotal = minPosTotal / 30;
    avgPos = (maxPosTotal + minPosTotal)/2;
    writeDAC16fast(avgPos);
  }
  if(count_1 >= 30){
    count_1++;
    nextSweep = true;
  }
  thresholdCount = 0;
  
  //--------------------Threshold Loop--------------------//
  while(nextSweep == true){
    v_init = 15000;
    while(next_2 == true){
      out00 = calculateOutput();
      out_2 = ((out00/4) + 2048); 
      writeDAC16fast(v_init);
      analogWrite(DAC1, out_2);
      shiftedOutput = out_2 - 2048; 
      threshold_1 = (2800 - 2048); 
      if((thresholdCount < 30) && (shiftedOutput > threshold_1 - 5) && (shiftedOutput < threshold_1 + 5) && (v_init > 15000 + 500) && (v_init < 32768 - 500)){ 
        thresholdCount++;
        samples.add(v_init);
        if(thresholdCount == 29){
          v_init = samples.getAverage();
          Serial.print(v_init); Serial.print(" V1 "); Serial.print('\n');
          v_initLock = v_init - 100; 
          gotoLock = 1;
          nextSweep = false;
          thresholdCount++;  
        }   
      }
      
      if(v_init < 32768 ){
        v_init = v_init + 10;
      }
      else{
        next_2 = false;
        v_init = 0;
      }
    }
    if(nextSweep == true){
      next_2 = true;
    }
    else if(nextSweep == false){
      next_2 = false;
    }
  }
  relockCount = 0;

  //--------------------Lock Loop--------------------//
  while(gotoLock == 1){

    if(lockCount < 5000){ //startLockCount < 10 &&
      kp = 0.01 ; //0.6 
      ki = 0.001 ; //0   
      kd = 0 ; //0.5 
    }

    out000 = calculateOutput();
    out_3 = ((out000/8) + 2048);
    outShifted = out_3 - 2048;
    error = outShifted - setpoint;
    
    accumulator = accumulator + error;
    diff = error - error1;
    error1 = error;
    Pterm = error * kp;
    Iterm = accumulator * ki;
    Dterm = diff * kd;
    
    if(accumulator > 2500){
      accumulator = 2500;
      Iterm = accumulator * ki;
      PIcontrol = Pterm + Iterm + Dterm;
    }
    if(accumulator < -2500){
      accumulator = -2500;
      Iterm = accumulator * ki;
      PIcontrol = Pterm + Iterm + Dterm;
    }
    else{
      PIcontrol = Pterm + Iterm + Dterm;
    }

    v_initLock = v_initLock + PIcontrol;
    
    writeDAC16fast(v_initLock);
    analogWrite(DAC0, error + 2048);

    lockCount++;  

  }
}

//=========================== FUNCTIONS: =======================================

//Generate locking signal
int calculateOutput(){  

  delayMicroseconds(20);
  sample1 = 0;
  sample2 = 0;
  
  REG_PIOB_SODR = 0x8000000; //digitalWrite(D13, 1);
  for (int i=0; i<a; i++){
    sample1new = readADC16fast();
    sample1 = sample1 + sample1new;
  }

  REG_PIOB_CODR = 0x8000000;   //digitalWrite(D13, 0);
  

  REG_PIOC_CODR = 0x400000;    //digitalWrite(D8, 0);  // 0 = Low, Freq = 224.926 MHz
  delayMicroseconds(20);

  REG_PIOB_SODR = 0x8000000;    //digitalWrite(D13, 1);
  for (int i=0; i<a; i++){
    sample2new = readADC16fast();
    sample2 = sample2 + sample2new;
  }

  REG_PIOB_CODR = 0x8000000;    //digitalWrite(D13, 0);

  output = (-12)*(sample1 - sample2); 
 
  REG_PIOC_SODR = 0x400000;   //digitalWrite(D8, 1); // 1 = High, Freq = 278.556 MHz
 
  return output;
}

*/
//Read the inputsignal from the 16bit Analog to Digital Converter:
int readADC16fast() {
Areg = PIOA->PIO_PDSR;
Breg = PIOB->PIO_PDSR;
Creg = PIOC->PIO_PDSR;
Dreg = PIOD->PIO_PDSR;
return ((Areg & 0x8000)>>14 | (Areg & 0x80000)>>9 | (Breg & 0x4000000)>>26 | (Breg & 0x200000)>>6 | (Creg & 0x0004)<<4 | (Creg & 0x0010)<<3 | (Creg & 0x0040)<<2 | (Creg & 0x2100)<<1 | (Creg & 0x80000)>>8 | (Creg & 0x20000)>>5 | (Creg & 0x8000)>>2 | (Dreg & 0x0002)<<1 | (Dreg & 0x0008) | (Dreg & 0x0600)>>5);
}

//Output to the 16bit Digital to Analog Converter
void writeDAC16fast(int val) {
   REG_PIOC_CODR = 0x14000000;  // set Pin 3 & 4 to LOW
  
  Areg = (~val & 0x8000)>>1 | (val&0x0800)>>4 | (val&0x0020)<<15;
  NAreg = (val & 0x8000)>>1 | (~val&0x0800)>>4 | (~val&0x0020)<<15;
  Breg = (val & 0x0001)<<14;
  NBreg = (~val & 0x0001)<<14;
  Creg = (val & 0x0002)<<11 | (val & 0x0004)<<12 | (val & 0x0008)<<13 | (val & 0x0010)<<14 | (val & 0x0040)<<3 | (val & 0x0080) | (val & 0x0100)>>3 | (val & 0x0200)>>6 | (val & 0x0400)>>9; 
  NCreg = (~val & 0x0002)<<11 | (~val & 0x0004)<<12 | (~val & 0x0008)<<13 | (~val & 0x0010)<<14 | (~val & 0x0040)<<3 | (~val & 0x0080) | (~val & 0x0100)>>3 | (~val & 0x0200)>>6 | (~val & 0x0400)>>9; 
  Dreg = (val & 0x1000)>>6 | (val & 0x2000)>>11 | (val & 0x4000)>>14;
  NDreg = (~val & 0x1000)>>6 | (~val & 0x2000)>>11 | (~val & 0x4000)>>14;

  REG_PIOA_SODR = Areg;
  REG_PIOA_CODR = NAreg;
  REG_PIOD_SODR = Dreg;
  REG_PIOD_CODR = NDreg;
  REG_PIOC_SODR = Creg;
  REG_PIOC_CODR = NCreg;
  REG_PIOB_SODR = Breg;
  REG_PIOB_CODR = NBreg;
  
  REG_PIOC_SODR = 0x14000000;  // set Pin 3 & 4 to HIGH
}
