#include<Servo.h>
#include<PID_v1.h>
#include<SoftwareSerial.h>
#include <HCSR04.h>

int set=6,neg=-24,pos=24,base=130;//Setpoint,Negative,Positive,Base values.neg shows tilt on other side of ultrasonic sensor
long cm1=set;                     //For filtering purpose
const double a=0.5;               //Exponential filter parameter
const int servoPin = 9;           //Servo pin
#define TRIGGER   11
#define ECHO      12
UltraSonicDistanceSensor distanceSensor(TRIGGER, ECHO);
 
float Kp = 2.5;                                                    //Initial Proportional Gain
float Ki = 0;                                                      //Initial Integral Gain
float Kd = 1;                                                    //Intitial Derivative Gain
double Setpoint, Input, Output, ServoOutput;                                       



PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE);           //Initialize PID object, which is in the class PID.
                                                                      
                                                                     
                                                                     
                                                                     
Servo myServo;                                                       //Initialize Servo.


void setup() {

  Serial.begin(9600);                                                //Begin Serial 
  myServo.attach(servoPin);                                          //Attach Servo

  Input = readPosition();                                            //Calls function readPosition() and sets the balls
                                                                     //  position as the input to the PID algorithm
                                                                     
  
 
  myPID.SetMode(AUTOMATIC);                                         //Set PID object myPID to AUTOMATIC 
  myPID.SetOutputLimits(neg,pos);                                   //Set Output limits to neg and pos degrees. 
}

void loop()
{
 
  Setpoint = set;                                                      //Give value for setpoint
  Input = readPosition();                                            
 
  myPID.Compute();                                                   //computes Output in range of neg to pos degrees
  
  ServoOutput=base+Output;                                            // value in base is my horizontal 
  myServo.write(ServoOutput);                                        //Writes value of Output to servo
  
  
}
      
      
      

float readPosition() {

  long duration, cm,cmn;
  unsigned long now = millis();
  //Cria variavel do tipo int
    int distancia = 0;
    
    //Variável recebe o valor da função da biblioteca
    distancia = distanceSensor.measureDistanceCm();
    
    //exibe na porta serial o valor de distancia medido
    Serial.println(distancia);

  cm = distancia;
  
  
  if(cm > 30)              // 30 cm is the maximum position for the ball
  {cm=30;}                //Signal Conditioning for ultrasonic sensor  

  cmn = a * cm + (1 - a) * cm1;     //Exponential filter- signal conditioning 
  Serial.print(cm); Serial.print("\t");
  Serial.println(cmn);            //cmn is filtered value
  delay(10);
  cm1 = cmn;                      //saved to cm1 which is used as history in exponential filter
  
  return (cmn);                           //Returns filtered distance value in cm
}