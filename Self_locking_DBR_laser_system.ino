// NOTE: HAVE NOT FINISHED CORRECTIONS TO CLEAN UP THE CODE, NEED TO COMMENT OUT THE COMMENTS PROPERLY <---- I think I have now

//  FILE: TheCode_090317 . ino
//  AUTHOR: Carolyn Cowdell
// VERSION: A
// PURPOSE: ’ Self−Locking ’  Laser Sweep Demodulation Lock Re− Lock
// CONTACT: ccowdell@gmail .com
//

//#include <MovingAverageFilter .h>
//#include <theCurrent .h>
/*For repump  transition  comment out theCurrent  and include theRepumpCurrent*/
//#include <theRepumpCurrent .h>
//#include <RunningMedian .h>
#include <SPI.h>
//#include <LDC.h>
#include <DDS.h>


//*StatLoop  can be  changed to  control how long  the broad sweep
//is The longer i t is run , the better the chance of finding the correct Locking range
const int statLoops = 30;
const int statLoops2 = 30;
//* variables  for pins  and switches  used*/
const int Switch = 3;
const int resetPin = 12;
const int f3GroundStatePin = 6;
const int RepumpPin = 8;
const int kiPin = A5; 29 const int kpPin = A4;
const int specPin = A1;
const int LEDPin = 2;
const int logicSweepPin = 2;

 //* variables */
  float avg = 0;
int f3GroundState = 0;
int Repump = 0;
int count_1 = 0;
float curValue = 0;
bool next = true ;
        float out = 0;
  float sweepAvg = 0;
  float sweepMax = −5000;
  float sweepMaxPos = 0;
  float sweepMin = 5000;
  float sweepMinPos = 0;
  float theMin = 0;
  float theMax = 0;
  float midpoint = 0;
  float current = 0;
  float dacEnd = 4095;
  float dacAdd = 1;
  float sweepMaxPosTotal = 0;
  float sweepMinPosTotal = 0;
  float currentLock = 0;
int gotoLock = 0;
int thresholdCount = 0;
bool nextSweep = true ;
bool next_2 = true ;
float shiftedOutput = 0;
float max_1 = 0;
float min_1 = 0;
float med_1 = 0;
float StartCurrent_1 = 0;
float max_2 = 0;
float min_2 = 0;
float med_2 = 0;
float StartCurrent_2= 0;
float max_3 = 0;
float min_3 = 0;
float med_3 = 0;
float StartCurrent_3 = 0;
float Accumulator = 0;
float PIControl = 0;
float currentLockTotal = 0;
float currentLockPrev = 0;
float errorTotal = 0;
float errorAvg = 0;
float currentLockAvg = 0;
float errorPrev = 0;
int relockCount = 0;
int repetitive = 0;
int lockCount = 0;
  float out_3 = 0;
  float outShifted = 0;
  float Setpoint = 0;
  float ki =0;
  float kp =0;
  float Error = 0;
  float minCurrent = 0;
  float maxCurrent = 0;
  float diffCurrent = 0;
  float sizeCurrent = 0;
  float integral = 0;
  float proportional = 0;
  float out_2 = 0;
int startLockCount = 0;
float PTerm = 0;
float ITerm = 0;
float milliVolts = 0; 
int threshold_1 ;
float theCurrent = 0;
float StartDiff = 0;
char currArray [ 8 ] ;
byte hexCurrent [10] = { 0x30 ,0 x31 ,0 x32 ,0 x33 ,0 x34 ,0 x35 ,0 x36 ,0 x37 ,0 x38 ,0 x39 };
String  strcurrent ;


//*Defines so  the device  can do a  s e l f reset */
#define SYSRESETREQ (1<<2)
#define VECTKEY (0x05fa0000UL) 112 #define VECTKEY_MASK (0 x0000ffffUL )
#define AIRCR (*( uint32_t *)0xe000ed0cUL) // fixed
arch−defined  address
#define REQUEST_EXTERNAL_RESET (AIRCR=(AIRCR&VECTKEY_MASK) | VECTKEY|SYSRESETREQ) 115

 //Use a running median of 15 samples for f i r s t 30 loops 119 RunningMedian samples = RunningMedian(15) ;
//Use a running median  of  15 samples  for threshold loop
RunningMedian samples2 = RunningMedian(15) ; 122 //Use a running median of 10 for Re−locking
  RunningMedian Current = RunningMedian(10) ;

//Moving  average f i l t e r of  30  for the long  sweep
MovingAverageFilter movingAverageFilter (30) ;
//Moving  average of  40  during  the short sweep ( looking for
threshold )
MovingAverageFilter movingAverageFilter2 (40) ;


LDC controlLDC ; // variable  for calling LDC library functions
DDS controlDDS ; // variable  for calling DDS library

 //*This function reads the potentiometer on A5 and calculates the integral gain*/ 135 float calcIntegral () {
integral = analogRead( kiPin ) ;
integral = integral * 0.000001;
return integral ;
}

 //* this function read the potentiometer of A4 and calculates the proportional gain*/ 142 float calcProportional () {
proportional = analogRead(kpPin) ;
proportional = proportional * 0.00001;
return proportional ;
}

void setup () {
//Open  Serial  communication with  Arduino 
Serial  port //
Serial . begin (9600) ;
//*Set the read and write resolution of the DACs to 12 bits , this is the maximum available */
analogReadResolution (12) ;
analogWriteResolution (12) ;
//*Setup pins on the Due that  are used  Input = read ,
Output=write */
pinMode(Switch , INPUT) ;
pinMode(LEDPin, OUTPUT) ;
pinMode( specPin , INPUT) ;
pinMode( kiPin , INPUT) ;
pinMode(kpPin , INPUT) ;
pinMode(A3, INPUT) ;
pinMode( logicSweepPin , OUTPUT) ;
pinMode( resetPin , OUTPUT) ;
pinMode( f3GroundStatePin , INPUT) ;
pinMode(RepumpPin , INPUT) ;

//*choose  which crossover resonance to  lock  to ,  either
F=3−F’3XF’4 f3GroundStatePin HI ,
F=3−F’2xF ’4  f3GroundStatePin LOW */
f3GroundState = digitalRead ( f3GroundStatePin ) ;
//* If you want to lock to F=2−F’1xF ’2 , the repump transition switch Repump to HI*/
Repump = digitalRead (RepumpPin) ;
//*Delay to  ensure  switches  are set */
delay (1000) ;
//*Set the i n i t i a l Current via serial  connection  to
LDC501
i f repump  switch  high ,  then  set to  230.00mA, i f repump switch is  low 234.50mA*/
if (Repump == 1) {
controlLDC . setCurrentRepump () ;
}
else {
controlLDC . setCurrent () ;
}
//* I n i t i a l i z e  the SPI of  the Arduino and i n i t i a l i z e the registers of  the AD9958*/
controlDDS . initializeDDS () ;

  }


void loop () {
//*Use the broad sweep*/
digitalWrite ( logicSweepPin ,LOW) ;
//**************************** SWEEP LOOP*************************************/
//* while  counter less  than  Statloops (150  for F=3 grounds states  and 30  for F=2 ground  states */

//*comment out while (count_1 < statLoops ) and uncomment while ( true ) i f you want to just run the
broad sweep
No thresholds , locking , or  re−locking */

// while ( true ){
while (count_1 < statLoops ) {
/*Set i n i t i a l DAC increment to 0*/
curValue = 0;

//*next ,  sweep through a l l 4096 DAC values */
next = true ;
while ( next == true) {
  
//* Calculate  Output ,  add output  to  the MA f i l t e r ,  and output  the DAC value ( sweep ) on DAC0 and output  ( demodulation ) on DAC1 */
if (Repump == 1) {
out = controlDDS .
calculateOutputRepump () ;
}
else {
out = controlDDS .
calculateOutput () ;
}
out = movingAverageFilter . process (out)
;
analogWrite (DAC0,  curValue ) ;
analogWrite (DAC1,  out) ;

//*Make sure values  that  are found aren’t anomalies at the start and finish of sweep ( usually there are spikes  there )*/
if ( curValue > 1500 && curValue <
3900) {
  
//* find the average value of the sweep*/
sweepAvg += out ;

//* find the max value and position  in  the sweep*/
if (out > sweepMax)
{
sweepMax = out ;
sweepMaxPos = curValue
; 
  }
//* find the min value and position  in  the sweep*/
if (out < sweepMin)
{
sweepMin = out ;
sweepMinPos = curValue ;
 }
}
//* If DAC value <4095, keep incrementing the sweep*/
if ( curValue < dacEnd) {
curValue = curValue + dacAdd ;
}
/* If sweep is  finished ,  reset and
start over and  long  as count_1  is less statloops . */
else {
next = false ;
  curValue = 0;
}
}
//*For a l l sweeps , repeatedly add the positons of the maximums and add the positions of the minimums*/
sweepMaxPosTotal += sweepMaxPos ; 243 sweepMinPosTotal += sweepMinPos ;
count_1++;
}
//*When broad  sweep loops have  finished ,  statloops = Max: */
if (count_1 == statLoops ) {

//* find the average position  of  the maximum values and average position of the minimum values */
sweepMaxPosTotal = sweepMaxPosTotal / statLoops ;
sweepMinPosTotal = sweepMinPosTotal /
statLoops ;
//*Add the average minimum position  and the average maximum position  and divide  by two*/
avg = (sweepMaxPosTotal + sweepMinPosTotal) /
2;
//* find the i n i t i a l currrent  for the small sweep*/

// NEED TO FINISH COMMENTING PIECES OF CODE OUT

/* First  convert the avg to a  voltage in mV*/
milliVolts = ((( avg*2.2 / 4095) − 0.55)
*0.0909) * 1000;
/*Next  convert the mV to a current */
current = ((0.0231* milliVolts ) + 235.6) ;
/*Print these to  serial  port  for loggging */
Serial . println ( milliVolts , 5) ;
Serial . println ( current ,  5) ;

/* find the i n i t i a l currrent  for the small
repump sweep*/
if (Repump == 1) {
/*Same process  as  above , however the
addition  is  less */
milliVolts = (((( avg + 250) *2.2 /
4095) − 0.55) *0.0909) * 1000;
current = ((0.0085* milliVolts ) +
231.50) ;
Serial . println ( milliVolts , 5) ;
Serial . println ( current ,  5) ;
}
/* convert  current into  a string  then a
character array*/
strcurrent = String ( current , 8) ;
strcurrent . toCharArray( currArray , 8) ;

/*Get each  character from  the character array
*/
char zero = currArray [ 0 ] ;
  char one = currArray [ 1 ] ;    
  char two = currArray [ 2 ] ;    
  char three = currArray [ 3 ] ;    
  char four = currArray [ 4 ] ;   
  char five = currArray [ 5 ] ;   
  char six = currArray [ 6 ] ;    
  char seven = currArray [ 7 ] ;    
  char eight = currArray [ 8 ] ;    
  /*change  the characters we need  to  integers */
  int zeroInt = zero − ’0 ’ ;   
  int oneInt = one − ’0 ’ ;   
  int twoInt = two − ’0 ’ ;   
  int fourInt = four − ’0 ’ ;   
  int fiveInt = five − ’0 ’ ;   
  int sixInt = six − ’0 ’ ;   
/* look up in hex array here  i f zeroInt = 5
then 0x35 w i l l be  returned */
byte hex1 = hexCurrent [ zeroInt ] ;
byte hex2 = hexCurrent [ oneInt ] ; 295 byte hex3 = hexCurrent [ twoInt ] ;
byte hex4 = hexCurrent [ fourInt ] ; 297  byte hex5 = hexCurrent [ fiveInt ] ;
byte hex6 = hexCurrent [ sixInt ] ;
delay (1000) ;
/*Write the hexadecimal version of  the current to  the LDC*/
/*open  serial  connection ,  delay , print , and
close */
Serial3 . begin (9600) ;
delay (200) ;
Serial . println ( " hi " ) ;
Serial3 . write (0x53) ;
Serial3 . write (0x49) ;
Serial3 . write (0x4C) ;
Serial3 . write (0x44) ;
Serial3 . write (hex1) ;
Serial3 . write (hex2) ;
Serial3 . write (hex3) ;S
erial3 . write (0x2E) ; 
Serial3 . write (hex4) ;
Serial3 . write (hex5) ;
Serial3 . write (hex6) ;
delay (50) ;
Serial3 . write (0x0D) ;
Serial3 . write (0x0A) ;
Serial3 . end () ;
  /*Print out , Min,  the current the loop  found and Max
  and avg for logging to  text  f i l e */
Serial . print ( "MID: ␣" ) ;
Serial . println ( current ) ;
Serial . print ( "Sweep␣Min: ␣" ) ;
Serial . println (sweepMinPosTotal) ; 326 Serial . print ( "Sweep␣Max: ␣" ) ;
Serial . println (sweepMaxPosTotal) ;
Serial . print ( "Sweep␣avg : ␣" ) ;
Serial . println (avg) ;
330
}
/* If count_1 is  greater than  statloops : */
if (count_1 >= statLoops ) {
count_1++;
/*Write a logic High  to  hardware  switch  to
change  to  small sweep*/
digitalWrite ( logicSweepPin , HIGH) ;
/* delay  to make sure  this  is  recieved  and
switch  has time  to  switch */
delay (500) ;
nextSweep = true ;
}
  thresholdCount = 0;
/******************THRESHOLD LOOP********************/
/*Keep sweeping unless nextSweep is false , or thresholdCounter_2 exceeds 90 Vales*/
while (nextSweep == true) {
/* t e l l  circuit to  use the small sweep again ,
just  to make sure*/
digitalWrite ( logicSweepPin , HIGH) ;
/* I n i t i a l i z e DAC value  and start sweep*/
curValue = 0;
// while ( true ){  while (next_2 == true) {
/* Calculate  output  and process by new
MA f i l t e r ,  Output  to DAC1 and DAC
Value ( sweep ) to DAC0*/
out_2 = controlDDS . calculateOutput () ;
if (Repump == 1) {
/*repump  is  calculated
s l i g h t l y d i f f e r e n t l y as  i t
is  multiplied  by a  higher gain*/
out_2 = controlDDS .
calculateOutputRepump () ;
}
out_2 = movingAverageFilter2 . process (
out_2) ;
/*Write current sweep to DAC0 and
write demodulation  to DAC1*/
analogWrite (DAC0,  curValue ) ;
analogWrite (DAC1, out_2) ;
364
/* s h i f t  output  such  that  the code  sees the  slope as  crossing  zero */
shiftedOutput = out_2 − 2500;
/* If 3 minutes have  passed  and the
transition  has not been  found , s h i f t the short sweep s l i g h t l y */
if ( millis ()==300000 ){
milliVolts = ((( avg*2.2 /
4095) − 0.55) *0.0909) * 1000; // convert DAC Value to mV
current = ((0.0231* milliVolts )
+ 235.2) ; // convert to current
Serial . println ( milliVolts , 5) ;
Serial . println ( current ,  5) ;
Serial . println ( "2" ) ;
/* convert  current into  a
string  then a char array*/
strcurrent = String ( current ,
8) ;
strcurrent . toCharArray(
currArray , 8) ;
/*Get each  character from  the
array*/
char zero = currArray [ 0 ] ;
char one = currArray [ 1 ] ;
char two = currArray [ 2 ] ;
char three = currArray [ 3 ] ;
char four = currArray [ 4 ] ;
char five = currArray [ 5 ] ; 
char six = currArray [ 6 ] ;
char seven = currArray [ 7 ] ;
char eight = currArray [ 8 ] ;
/*change  the characters  to integers */
  int zeroInt = zero − ’0 ’ ;
  int oneInt = one − ’0 ’ ;
  int twoInt = two − ’0 ’ ;
  int fourInt = four − ’0 ’ ;
  int fiveInt = five − ’0 ’ ;
  int sixInt = six − ’0 ’ ;
/* look up in hex array here  i f
zeroInt = 5 then 0x35 w i l l be returned */
byte hex1 = hexCurrent [ zeroInt
] ;
byte hex2 = hexCurrent [ oneInt
] ;
byte hex3 = hexCurrent [ twoInt
] ;
byte hex4 = hexCurrent [ fourInt
] ;
byte hex5 = hexCurrent [ fiveInt
] ;
byte hex6 = hexCurrent [ sixInt
] ;
delay (1000) ;
/*Write the hexadecimal
version of  the current to
the LDC*/
Serial3 . begin (9600) ;
delay (200) ;
Serial . println ( " hi " ) ;
Serial3 . write (0x53) ;
Serial3 . write (0x49) ;
Serial3 . write (0x4C) ;
Serial3 . write (0x44) ;
Serial3 . write (hex1) ;
Serial3 . write (hex2) ;
Serial3 . write (hex3) ;
Serial3 . write (0x2E) ;
Serial3 . write (hex4) ;
Serial3 . write (hex5) ;
Serial3 . write (hex6) ;
delay (50) ;
Serial3 . write (0x0D) ;
Serial3 . write (0x0A) ;
Serial3 . end () ;
}
/* If 6 minutes have  passed  and the
transition  has not been  found , s h i f t the short sweep s l i g h t l y */
if ( millis ()==5200000 ) {
milliVolts = ((( avg*2.2 /
4095) − 0.55) *0.0909) * 1000; // convert DAC Value to mV
current = ((0.0231* milliVolts )
+ 235.5) ; // convert to current
Serial . println ( milliVolts , 5) ;
Serial . println ( current ,  5) ;
Serial . println ( "3" ) ;
/* convert  current into  a
string  then a char array*/
strcurrent = String ( current ,
8) ;
strcurrent . toCharArray(
currArray , 8) ;
433
/*Get each  character from  the
array*/
char zero = currArray [ 0 ] ;
char one = currArray [ 1 ] ;
char two = currArray [ 2 ] ;
char three = currArray [ 3 ] ;
char four = currArray [ 4 ] ; 
char five = currArray [ 5 ] ;
char six = currArray [ 6 ] ;
char seven = currArray [ 7 ] ;
char eight = currArray [ 8 ] ;
/*change  the characters  to
integers */
  int zeroInt = zero − ’0 ’ ;
  int oneInt = one − ’0 ’ ;
  int twoInt = two − ’0 ’ ;
  int fourInt = four − ’0 ’ ;
  int fiveInt = five − ’0 ’ ;
  int sixInt = six − ’0 ’ ;
/* look up in hex array here  i f
zeroInt = 5 then 0x35 w i l l be returned */
byte hex1 = hexCurrent [ zeroInt
] ;
byte hex2 = hexCurrent [ oneInt
] ;
byte hex3 = hexCurrent [ twoInt
] ;
byte hex4 = hexCurrent [ fourInt
] ;
byte hex5 = hexCurrent [ fiveInt
] ;
byte hex6 = hexCurrent [ sixInt
] ;
delay (1000) ;
/*Write the hexadecimal
version of  the current to
the LDC*/
Serial3 . begin (9600) ;
delay (200) ;
Serial . println ( " hi " ) ;
Serial3 . write (0x53) ;
Serial3 . write (0x49) ;
Serial3 . write (0x4C) ;
Serial3 . write (0x44) ;
Serial3 . write (hex1) ;
Serial3 . write (hex2) ;
Serial3 . write (hex3) ;
Serial3 . write (0x2E) ;
Serial3 . write (hex4) ;
Serial3 . write (hex5) ;
Serial3 . write (hex6) ;
delay (50) ;
Serial3 . write (0x0D) ;
Serial3 . write (0x0A) ;
Serial3 . end () ;
}
/* If 9 minutes have  passed  and the
transition  has not been  found , s h i f t the short sweep s l i g h t l y */
if ( millis ()==720000  ) {
milliVolts = ((( avg*2.2 /
4095) − 0.55) *0.0909) * 1000; // convert DAC Value to mV
current = ((0.0231* milliVolts )
+ 235.9) ; // convert to current
Serial . println ( milliVolts , 5) ;
Serial . println ( current ,  5) ;
Serial . println ( "4" ) ;
/* convert  current into  a
string  then a char array*/
strcurrent = String ( current ,
8) ;
strcurrent . toCharArray(
currArray , 8) ;
489
/*Get each  character from  the
array*/
char zero = currArray [ 0 ] ;
char one = currArray [ 1 ] ;
char two = currArray [ 2 ] ;
char three = currArray [ 3 ] ;
char four = currArray [ 4 ] ;
char five = currArray [ 5 ] ;
char six = currArray [ 6 ] ;
char seven = currArray [ 7 ] ;
char eight = currArray [ 8 ] ;
/*change  the characters  to
integers */
  int zeroInt = zero − ’0 ’ ;
  int oneInt = one − ’0 ’ ;
  int twoInt = two − ’0 ’ ;
  int fourInt = four − ’0 ’ ;
  int fiveInt = five − ’0 ’ ;
  int sixInt = six − ’0 ’ ;
/* look up in hex array here  i f
zeroInt = 5 then 0x35 w i l l be returned */
byte hex1 = hexCurrent [ zeroInt
] ;
byte hex2 = hexCurrent [ oneInt
] ;
byte hex3 = hexCurrent [ twoInt
] ;
byte hex4 = hexCurrent [ fourInt
] ;
byte hex5 = hexCurrent [ fiveInt ] ;
byte hex6 = hexCurrent [ sixInt
] ;
delay (1000) ;
/*Write the hexadecimal
version of  the current to
the LDC*/
Serial3 . begin (9600) ;
delay (200) ;
Serial . println ( " hi " ) ;
Serial3 . write (0x53) ;
Serial3 . write (0x49) ;
Serial3 . write (0x4C) ;
Serial3 . write (0x44) ;
Serial3 . write (hex1) ;
Serial3 . write (hex2) ;
Serial3 . write (hex3) ;  
Serial3 . write (0x2E) ;
Serial3 . write (hex4) ;
Serial3 . write (hex5) ;
Serial3 . write (hex6) ;
delay (50) ;
Serial3 . write (0x0D) ;  
Serial3 . write (0x0A) ;
Serial3 . end () ;
}
if ( millis () == 930000) {
delay (1000) ;
REQUEST_EXTERNAL_RESET; //
Arduino Due code  reset
}
/*Threshold 1 the calculated  output
must be within plus or minus 5 of this point and the position of the corresponding  output
in  the sweep must not  be  too close to
the start or end of the sweep as again the MA f i l t e r can create
false spikes  here*/
/*Threshold for repump and  crossover 1
are s l i g h t l y different */
if (Repump == 1) {
threshold_1 = 500;
}
else {
threshold_1 = 800;
}
if (( thresholdCount < 30) && (
shiftedOutput < threshold_1 + 5) &&
( shiftedOutput > threshold_1 − 5) && ( curValue > 0 + 500) && ( curValue < dacEnd − 500) )
{
/* If value is  found increment
thresholdCount , add i t to a running average , print the position and value so that
they  can be  logged */
thresholdCount++;
samples . add( curValue ) ;
Serial . print ( curValue ) ;
Serial . print ( ’ , ’ ) ;
Serial . println ( shiftedOutput ) ;
/* If appropriate values  have
been found 30 times , get the average , median , min ,
and max of  running average
*/
if ( thresholdCount == 29) {
curValue = samples .
getAverage () ;
max_1 = samples . getHighest () ;
min_1 = samples . getLowest () ;
med_1 = samples .
getMedian () ;
Serial . println ( ’␣ ’ ) ;
Serial . println ( " after ␣
30:␣" ) ;
Serial . print ( curValue )
;
Serial . print ( ’ , ’ ) ;
Serial . print (max_1) ; 
Serial . print ( ’ , ’ ) ;
Serial . print (min_1) ; 
Serial . print ( ’ , ’ ) ;
Serial . print (med_1) ;
Serial . println ( ’␣ ’ ) ;
/*This  is  passed  to
the next  loop  or lock loop */
StartCurrent_1 = med_1
;
/*read  the pin
choosing between F ’=3−F’=4 or F’=2−F ’=4 crossover transition from F=3
ground  state */
f3GroundState =
digitalRead (
f3GroundStatePin ) ;
delay (100) ;
/* If switch  is  high , F
’=3−F’=4 has been chosen and now exit threshold loop */
/* i f  Repump switch is
high , repump has been chosen and
exit  threshold loop
*/
  if  ( f3GroundState == 1 && Repump == 0) {
    /* value shifted
down
s l i g h t l y to
ensure
correct peak  is found*/
    currentLock =
StartCurrent_1 − 10;
    gotoLock = 1;
    nextSweep = false ;
  } 
  if  (Repump == 1) {
    /* value
shifted
down
s l i g h t l y to
ensure correct peak is found*/
currentLock =
StartCurrent_1
200;
gotoLock = 1; 592 nextSweep =
false ;

}
thresholdCount++;
}
}
/*Threshold 2 the calculated  output
must be within  plus  or minus 5  of
this point and the position of the corresponding output
in  the sweep must be greater than  the
minimum found in threshold 1 and not to close to the end of the sweep as again the MA f i l t e r can
create  false spikes  here*/
if (( thresholdCount < 60) && (
thresholdCount >= 30) && ( shiftedOutput < −1500 + 5) && ( shiftedOutput > −1500 − 5) && ( curValue > StartCurrent_1 ) && (
curValue < dacEnd − 200) )
{
/* If value is  found increment
thresholdCount , add i t to a running average , print the position and value so that
they  can be  logged */
thresholdCount++;
samples . add( curValue ) ;
Serial . print ( curValue ) ;
Serial . print ( ’ , ’ ) ;
Serial . println ( shiftedOutput ) ;
608
/* If appropriate values  have
been found 30 times , get the average , median , min ,
and max of  running average
*/
if ( thresholdCount == 59) {

curValue = samples .
getAverage () ;
max_2 = samples . getHighest () ;
min_2 = samples . getLowest () ;
med_2 = samples .
getMedian () ;
Serial . println ( ’␣ ’ ) ;
Serial . println ( " after ␣
30:␣" ) ;
Serial . print ( curValue )
;
Serial . print ( ’ , ’ ) ;
Serial . print (max_2) ; 
Serial . print ( ’ , ’ ) ;
Serial . print (min_2) ;
Serial . print ( ’ , ’ ) ;
Serial . print (med_2) ;
Serial . println ( ’␣ ’ ) ;
/*This  is  passed  to
the next  loop */
StartCurrent_2 = med_2 ;
thresholdCount++;

}
}
/*Threshold 2 the calculated  output
must be within  plus  or minus 5  of
this point and the position of the corresponding output
in  the sweep must be greater than  the
minimum found in threshold 2 and not to close to the end of the sweep as again the MA f i l t e r can
create  false spikes  here*/
if (( thresholdCount < 90) && (
thresholdCount >= 60) && ( shiftedOutput < 500 + 5) && ( shiftedOutput > 500 − 5) && ( curValue > StartCurrent_2 ) && (
curValue < dacEnd − 200) )
{

thresholdCount++;
samples . add( curValue ) ;
Serial . print ( curValue ) ;
Serial . print ( ’ , ’ ) ;
Serial . println ( shiftedOutput ) ;

/* If appropriate values  have
been found 30 times , get the average , median , min ,
and max of running average and print for log */
if ( thresholdCount == 89) {
645
curValue = samples .
getAverage () ;
max_3 = samples . getHighest () ;
min_3 = samples . getLowest () ;
med_3 = samples .
getMedian () ;
Serial . println ( ’␣ ’ ) ;
Serial . println ( " after ␣
30:␣" ) ;
Serial . print ( curValue )
;
Serial . print ( ’ , ’ ) ;
Serial . print (max_3) ; 
Serial . print ( ’ , ’ ) ;
Serial . print (min_3) ;
Serial . print ( ’ , ’ ) ;
Serial . print (med_3) ;
Serial . println ( ’␣ ’ ) ;
/*read  the pin
choosing between F ’=3−F’=4 or F’=2−F ’=4 crossover transition from F=3
ground  state */
f3GroundState =
digitalRead (
f3GroundStatePin ) ;
delay (100) ;
/* If switch  is  high , F
’=2−F’=4 has been chosen and now exit threshold loop */
if ( f3GroundState ==
{
StartCurrent_3
= med_3;
/* value
shifted up
to  ensure
correct peak  is found*/
currentLock =
curValue +
100;
gotoLock = 1; 669 nextSweep =
false ;
  }

  thresholdCount++;

}
}

/*Keep incrementing sweep until told
otherwise */
if ( curValue < dacEnd) {
curValue = curValue + dacAdd ;
}
/*Keep sweeping until told  otherwise */
else {
next_2 = false ;
curValue = 0;
}
}
if (nextSweep == true) {
next_2 = true ;
}
else if (nextSweep == false ) {
next_2 = false ;
}
}

Serial . println ( currentLock ) ;
Serial . println ( Setpoint ) ;
/* i n i t i a l i z e  for Lock Loop , make sure short sweep is being  used*/
relockCount = 0;
digitalWrite ( logicSweepPin , HIGH) ;

/*********THE LOCK LOOP*****************/
/*Once value  is  passed  to  this  loop ,  code  stays in
this  loop  unless  a reset to  the Arduino is  called
to  re−lock */
while ( gotoLock == 1) {
/*When just locking to a point ,  keep  ki  and kp small */
if ( startLockCount < 10 && lockCount < 5000) {
ki = 0.000001;
kp = 0.000001;
}
else if ( startLockCount >= 10) {
ki = calcIntegral () ;
kp = calcProportional () ;
}


  /* Calculate  output ,  s h i f t output  so  code  sees
zero−crossing linear  slopes  crossing  zero */

if (Repump == 1) {
out_3 = controlDDS . calculateOutputRepump () ;
}
else {
out_3 = controlDDS . calculateOutput () ;
}
outShifted = out_3 − 2500;
/*Error : how far the current is  from  the set−
point 0. */
Error = outShifted − Setpoint ;

/*Again , i f just  locking to  the point , make
sure error value doesn ’ t exceed 100 , otherwise the
code  could lock  to  the wrong transition */
if (( startLockCount < 10) && ( Error ) > 100) {
Error = 100;
}
if (( startLockCount < 10) && ( Error ) < −100) {
Error = −100;
}
/*PI CONTROLLER*/
/*Add integral  error , then  add proportional
error */

Accumulator += Error ; // accumulator is  sum of
errors
PTerm = Error * kp ;  //  find  proportional gain
ITerm = ki*Accumulator ; // add integral  gain and  error accumulation
/* r e s t r i c t  accumulator so  that  integral  gain
does  not wind−up*/
if (Accumulator > 2500) {

Accumulator = 2500;
ITerm = ki*Accumulator ;
PIControl = ITerm + PTerm;

}
if (Accumulator < −2500) {
Accumulator = −2500;
ITerm = ki*Accumulator ;
PIControl = ITerm + PTerm;
}
else {
PIControl = ITerm + PTerm;
}

currentLock = currentLock + PIControl ; //add
PI  correction  to  previous  current
/* print  Error to DAC1 and new current to DAC0
*/
analogWrite (DAC0,  currentLock ) ;
analogWrite (DAC1,  Error + 2500) ;

/* i f  less  than  5000  loops have  passed ,  add
currents  and add errors */
if ( lockCount < 5000) {
currentLockTotal += currentLockPrev ;
errorTotal += errorPrev ;
}

errorPrev = Error ;
currentLockPrev = currentLock ;
/*When 5000 loops have  passed : */
if ( lockCount == 5000) {
startLockCount++;
relockCount++;
/* find average error and current of
the l a s t 5000  loops */

currentLockAvg = currentLockTotal /
lockCount ;
errorAvg = errorTotal / lockCount ;

/*add average current to an array of
10  values */
Current . add( currentLockAvg ) ;
minCurrent = Current . getLowest () ; 
// get
the lowest  of  l a s t 10  values
maxCurrent = Current . getHighest () ; //
get the highest of  l a s t 10  values
diffCurrent = abs(maxCurrent −
minCurrent) ; 
// calculate  absolute
difference  between the max and min.
sizeCurrent = Current . getSize () ; //make
sure  there are 10  values  in running  average

/* i f  the current difference  between
the max and min of  the past  10
values  exceeds 209.5 ,
the lock  has been  pushed  too far too
fast . The laser needs to be re− locked */
if ( diffCurrent > 209.5 && sizeCurrent
== 10 && Repump == 0) {
Serial . print ( " Error " ) ;
Serial . print ( " , " ) ;
Serial . print ( diffCurrent ) ;
Serial . println ( "/" ) ;
digitalWrite (52 , HIGH) ;
Serial . println ( " resetting " ) ;
delay (1000) ;
REQUEST_EXTERNAL_RESET; // Arduino Due code reset

}
/*Output  time  passed  since code
started , the ki  and kp  values ,  the
accumulator value , the current
difference now, the average error of the l a s t 5000 loops ,
and average current of  the l as t  5000
loops */
Serial . print ( millis () ) ;
Serial . print ( " ,␣" ) ;
Serial . print ( ki , 6) ;
Serial . print ( " ,␣" ) ;
Serial . print (kp ,  6) ;
Serial . print ( " ,␣" ) ;
Serial . print (Accumulator ,6) ; 810 Serial . print ( " ,␣" ) ;
Serial . print (minCurrent ,  6) ;
Serial . print ( " ,␣" ) ;
Serial . print ( diffCurrent ,  6) ;
Serial . print ( " ,␣" ) ;
Serial . print ( errorAvg , 3) ;
Serial . print ( " ,␣" ) ;
Serial . println (currentLockAvg ,  3) ;

/* reset values */ 
lockCount = 0;
currentLockAvg = 0;
errorAvg = 0;
errorTotal = 0;
currentLockTotal = 0;

Serial1 . begin (9600) ;
Serial1 . println ( "Run" ) ;
Serial1 . end () ;

}
lockCount++;
  }//end  of  Lock While
}//end  of  loop
  //END OF FILE

