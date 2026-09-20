#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <Wire.h>
#include <Servo.h>

#define maxPid 100

RF24 radio(9,10);

Servo PWM1;
Servo PWM2;
Servo PWM3;
Servo PWM4;

const uint64_t pipes[2] = {0xF0F0F0F0E1LL,0xF0F0F0F0E2LL};
int tx[16];
int rx[16];

boolean radioStatus = 1;
boolean timeOut = 0;
long long startTime;



//read joystic deaitlse
int joyX1,joyY1,joyX2,joyY2;

int x,y;

//-----------------pid -----------------

int pwm_width_1 = 0;
int pwm_width_2 = 0;
int pwm_width_3 = 0;
int pwm_width_4 = 0;

int32_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;
 
float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];

float elapsedTime, t, timePrev;
int i;
float rad_to_deg = 180/3.141592654;

float PIDx, PIDy, motor1,motor2,motor3,motor4;
float previousXerror, previousYerror;
float pid_p=0;
float pid_i_x=0;
float pid_i_y=0;
float pid_d=0;

double kp=3.55; // PID tuning values documented in the 2018 project report
double ki=0.003; // PID tuning values documented in the 2018 project report
double kd=2.05; // PID tuning values documented in the 2018 project report

int throttle;//=1100; //initial value of throttle to the motors
int yaw = 0;
float xDesiredAngle = 0; 
float yDesiredAngle = 0; 
float xError, yError;
unsigned long now;



void setup(void)
{

  Serial.begin(9600);

  radio.begin();



    radio.openReadingPipe(1,pipes[0]);
    radio.openWritingPipe(pipes[1]);
    
    radio.setPALevel(RF24_PA_MAX);
    radio.setRetries(2,15);
    radio.startListening();    
   radioStatus = 1;

   Wire.begin(); //begin the wire comunication
   
   PWM1.attach(3);
   PWM2.attach(4);
   PWM3.attach(5);
   PWM4.attach(6);

   t = millis();

   PWM1.writeMicroseconds(1000); 
   PWM2.writeMicroseconds(1000);
   PWM3.writeMicroseconds(1000); 
   PWM4.writeMicroseconds(1000);
 
   delay(6000); 
   
}

void loop(void)
{
      radio.startListening();   
      rx[15] = 0;
      timeOut = 0; 
//   receiver mode       
      while(radioStatus == 1){       
              while(radio.available())
              {
                   //start of receiver mode code
                   
                    radio.read(rx, sizeof(rx));  

              }
               getFlightData();
               getDeltaT();
               readAccelAndGyro();
               calculateAngle();
               calculateError();
               calculatePID();
               setMotorSpeed();
               updateError();
              
              if(rx[15] == 1234)
                   radioStatus = 0;
      }
      
 //  transmitter mode 
     timeOut = 0;
       if(radioStatus ==  0){
             radio.stopListening();
             

             startTime = millis();
             Wire.beginTransmission(8);
             Wire.requestFrom(8, 20); 
             int m=0;
             while(Wire.available()){           
                  tx[m] = Wire.read()<<8| Wire.read(); 
                  m++;
             }

            Wire.endTransmission();    // stop transmitting
   
               while(!radio.write(tx,sizeof(tx))){
                     if(millis()- startTime > 100)
                         break;
               }
             radioStatus = 1;
             

      }        
}



פונקציות מסטר (Void):

פונקציית קבלת נתוני טיסה :
void getFlightData()
{
     xDesiredAngle = rx[2];
     yDesiredAngle = rx[3];
     throttle = rx[0];
     yaw = rx[1]; 
    
}



פונקציית חישוב הפרש הזמן (דלתא t): 
void getDeltaT()
{
     now = millis();
     timePrev = t;  // the previous time is stored before the actual time read
     t = millis();  // actual time read
     elapsedTime = (t - timePrev) / 1000; 
}











פונקציית קריאת נתוני אקסלומטר וג'יירוסקופ:
void readAccelAndGyro()
{
      Acc_rawX = 0;Acc_rawY = 0;Acc_rawZ = 0;

      Wire.beginTransmission(0x68);
      Wire.write(0x3B); //Ask for the 0x3B register- correspond to AcX
      Wire.endTransmission(false);
      Wire.requestFrom(0x68,6,true); 
     
      Acc_rawX+=Wire.read()<<8|Wire.read(); //each value needs two registres
      Acc_rawY+=Wire.read()<<8|Wire.read();
      Acc_rawZ+=Wire.read()<<8|Wire.read();
     
      Gyr_rawX = 0;
      Gyr_rawY = 0; 
    
      Wire.beginTransmission(0x68);
      Wire.write(0x43); //Gyro data first adress
      Wire.endTransmission(false);
      Wire.requestFrom(0x68,4,true); //Just 4 registers
       
      Gyr_rawX+=Wire.read()<<8|Wire.read(); //Once again we shif and sum
      Gyr_rawY+=Wire.read()<<8|Wire.read();
      
      /*---X---*/
      Gyro_angle[0] = Gyr_rawX/131.0; 
      /*---Y---*/
      Gyro_angle[1] = Gyr_rawY/131.0;      
}

פונקציית חישוב זווית לפי אקסלומטר והג'יירו בעזרת מסנן : 
void calculateAngle()
{
       /*---X---*/
      Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
      /*---Y---*/
      Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;

      /*---X axis angle---*/
      Total_angle[0] = 0.98 *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle[0];
      /*---Y axis angle---*/
      Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1];  
}



פונקציית חישוב שגיאת הזווית :
void calculateError()
{
      xError = Total_angle[0] - xDesiredAngle;
      yError = Total_angle[1] - yDesiredAngle;  
}




פונקציית חישוב הPID (חישוב ערך הפרופורציונאלי+סכימת ערך האינטגרציה+חישוב ערך הדיפרנציאל)
void calculatePID()
{
      pid_p = kp*xError;
   
      if((xError<3)&&(xError>-3))
          pid_i_x = pid_i_x+(ki*xError);  
      
      pid_d = kd*((xError - previousXerror)/elapsedTime);
     
      PIDx = pid_p + pid_i_x + pid_d;
      
      if(PIDx < -maxPid)
         PIDx=-maxPid;
      else if(PIDx > maxPid)
         PIDx=maxPid;
 
      ////////////// PID Y axis //////////////////////////
      pid_p = kp*yError;
      
      if((yError<3)&&(yError>-3))
          pid_i_y = pid_i_y+(ki*yError);  

      pid_d = kd*((yError - previousYerror)/elapsedTime);
      
      PIDy = pid_p + pid_i_y + pid_d;
      
      if(PIDy < -maxPid)
          PIDy=-maxPid;
      else if(PIDy > maxPid)
          PIDy = maxPid;
}
פונקציית עדכון מהירות מנועים 
void setMotorSpeed()
{
      motor1 = throttle + PIDx + PIDy;           
      motor2 = throttle - PIDx - PIDy;
      motor3 = (throttle + PIDx - PIDy);
      motor4 = throttle - PIDx + PIDy;

      if(motor1 < 1100)
         motor1= 1100;
      else if(motor1 > 2000)
          motor1=2000;
      
      if(motor2 < 1100)
         motor2= 1100;
      else if(motor2 > 2000)
          motor2=2000;
      
      if(motor3 < 1100)
         motor3= 1100;
      else if(motor3 > 2000)
          motor4=2000;
      
      if(motor4 < 1100)
         motor4= 1100;
      else if(motor4 > 2000)
          motor4=2000;

      PWM1.writeMicroseconds(motor1);
      PWM2.writeMicroseconds(motor2);
      PWM3.writeMicroseconds(motor3);
      PWM4.writeMicroseconds(motor4);
}
פונקציית עדכון שגיאה :
void updateError()
{
      previousXerror = xError; //Remember to store the previous error.
      previousYerror = yError; //Remember to store the previous error.
}
