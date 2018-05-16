
/* Encoder Library - Basic Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */

#include <Encoder.h>
// dc motor 
#define enA 9
#define in1 7
#define in2 8

int rotDirection = 0;

// rpm ve pid
int set=160; 
float motor;
int count=0;
int start=0;
float rpm;
int lastTime;
int SampleTime=200;
double errSum;
double lastInput;
double Output;
double kp=0.1, ki=0.5, kd=1;
int PID_out;
// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder myEnc(5, 6);
//   avoid using pins with LEDs attached
unsigned long time_start=0;
unsigned long time_stop;
void setup() {
  // dc motor
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  
  // Set initial rotation direction
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  //
  Serial.begin(9600);
  Serial.println("Basic Encoder Test:");
}

long oldPosition  = 0;

void loop() 
{
 
  long newPosition = myEnc.read();
  motor=devir(newPosition);
 // Serial.println(devir(newPosition));
  // dc motor sür
  //int potValue = analogRmead(A0); // Read potentiometer value
  int pwmOutput = map(PID_out,0, 1023, 45 , 255); // Map the potentiometer value from 0 to 255
  analogWrite(enA, pwmOutput); // Send PWM signal to L298N Enable pin

  Compute (set,motor);
   Serial.print(300); Serial.print("\t");
    Serial.print(0); Serial.print("\t");
    Serial.print(160); Serial.print("\t");
  Serial.print(rpm); Serial.print("\t");
Serial.println();
 
  //Serial.println(rpm);
  
}

float devir(long new_Position)
{
  if ((new_Position != oldPosition)  )
  {
    if ((new_Position-oldPosition)>=1920 ||(new_Position-oldPosition)<=-1920)
    {
     count++;
    if ( count!=start)
      {
        time_stop=millis();
        rpm=(60000/(time_stop-time_start));
        time_start=time_stop;
       
      }
     oldPosition = new_Position;
     
    }
   }
    return rpm;
}
void Compute(double set, double Input)
{
  unsigned long now = millis();
   int timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      /*Compute all the working error variables*/
      double error = set - Input;
      errSum += error;
      double dInput = (Input - lastInput);
 
      /*Compute PID Output*/
      Output = kp * error + ki * errSum - kd * dInput;
      PID_out=Output;
if(PID_out>=1023)
{
    PID_out = 1023;
}
  if(PID_out<=-1023)
  {
    PID_out = -1023;
  }
  /*Remember some variables for next time*/
      lastInput = Input;
      lastTime = now;
   }
}
void SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}
