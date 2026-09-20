#include <Wire.h>

//LED yellow Red pins
int Red = 4;
int yellow = 5;

//---------------------------------------------mpu6050------------------------------------
int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;
 
float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];

float elapsedTime, time, timePrev;
int i;
float rad_to_deg = 180/3.141592654;

#include "U8glib.h"
U8GLIB_ST7920_128X64_1X u8g(53);	

//rf liberays:
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
RF24 radio(7,6);  // CE, CSN






#define    DA  26
#define    D0  22 //d0A
#define    D1  23 //d0B
#define    D2  24 //d0C
#define    D3  25 //d0D


//rf pins

const uint64_t pipe[2] = {0xF0F0F0F0E1LL,0xF0F0F0F0E2LL};
int tx[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1234}; 
int rx[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int throttle, pitch, yaw, roll;

 
const int joySensetivity = 50;

int  joyX1  , joyY1   , joyX2   , joyY2   ;
int  osJoyX1  , osJoyY1   , osJoyX2    , osJoyY2   ;
int  key  ;
int state = 2;
boolean radioStatus = 0;
boolean timeOut = 0;
long long startTime;








void   setup()
{
  //LED pins
  pinMode( Red , INPUT);
  pinMode( yellow , INPUT);  //__________________________________________________________

  Wire.begin(); //begin the wire comunication
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Serial.begin(19200);


  
  pinMode( DA , INPUT);
  pinMode( D0 , INPUT);  
  pinMode( D1 , INPUT);
  pinMode( D2 , INPUT);
  pinMode( D3 , INPUT);
  pinMode(53,OUTPUT);



  osJoyX1   = analogRead     ( A0 );
  osJoyY1   = analogRead     ( A1 );
  osJoyX2   = analogRead     ( A2 );
  osJoyY2   = analogRead     ( A3 );  

  radio.begin();
  radio.openWritingPipe(pipe[0]);
  radio.openReadingPipe(1,pipe[1]);
  radio.setPALevel(RF24_PA_MAX);
  radio.setRetries(2,15); 


  tx[15] = 0;
  throttle = 1100;
  pitch = 0;
  roll = 0;
  yaw = 0;

  u8g.setRot180();
  u8g.setColorIndex(1);      
  draw();  

u8g.begin();

}















void  loop()
{   
  analogWrite(Red,255);
  analogWrite(yellow,0);
  
  // --------------------------- transmitter mode ---------------------------------------//
    if(radioStatus ==  0){  

          startTime = millis();
          timeOut = 0;
            while(timeOut == 0)
            {

//transmitter mode
                if(digitalRead(DA)==1)
                {
                    readKey();
     //              draw();          
                }
                  if(state == 2) // joystic mode
                  {
                  readJoy();
                   throttle += joyY1;
                    if(throttle > 2000)
                        throttle = 2000;
                     else if (throttle <1100)
                         throttle  = 1100;
                   tx [0]  = throttle;
                   
                   yaw =joyX1*10;
                   tx [1] = yaw;
                   
                   pitch =  joyX2*10;
                   tx [2] = pitch;
                   
                   roll = joyY2*10;
                   tx [3] = roll;
                   
                   tx[14]=2;
          //          draw();
                   }
                  else if(state == 1) // mpu6050 mode
                  {
                    mpu6050();
                    int x,y,X,Y;
                    y= Total_angle[0] ;
                    x=Total_angle[1] ;
                    if(Y>-45 && y<-1) Y=1;
                    else if(Y>-2 && y<53) Y=2;
                    
                     if(x>4 && x<66) X=3;
                    else if(x>-55&& x<3) X=4;
                                        
                  tx[4]=Y;
                  tx[5]= X;
                  
                  tx[14]=1;

                  }                
                              
                
   //------------------end part of the code for transmition code -------------------
                  radio.write(tx, sizeof(tx));
                


  if(millis()- startTime >1000)
                  {
                       timeOut = 1;
                       tx[15] = 1234;
                       radio.write(tx, sizeof(tx)); // check if the transmit is ok
                       radioStatus = 1;
                       tx[15] = 0;
                       analogWrite(yellow,255);
                       analogWrite(Red,0);

                  }
            }
            draw(); 
    }
  // --------------------------- receiver mode ---------------------------------------//    
  if(radioStatus ==  1){
             radio.startListening();
// receiver mode  
         startTime = millis();
         timeOut = 0;

         while(radioStatus ==  1){       
                timeOut = 0;
                while(timeOut == 0) 
                {
    
                      if( millis() - startTime>100){
                          timeOut = 1;
                          radioStatus = 0;
                      }
                     


 if(radio.available() )
                      { 
                        analogWrite(yellow,255);
                        analogWrite(Red,0);

                            while(!radio.read(rx, sizeof(rx)));
                            radioStatus = 0;
                            timeOut = 1;

                                                   }
                }
              
     }
       radio.stopListening(); 
  }


 }














פונקציות שלט (Void):

פונקציית MPU6050 :
//////////////////////--------------  LOOP----------------///////////////////////
void mpu6050()
{
timePrev = time; // the previous time is stored before the actual time read
time = millis();  // actual time read
elapsedTime = (time - timePrev) / 1000; 
   
     Wire.beginTransmission(0x68);
     Wire.write(0x3B); //Ask for the 0x3B register- correspond to AcX
     Wire.endTransmission(false);
     Wire.requestFrom(0x68,6,true); 
 
 Acc_rawX=Wire.read()<<8|Wire.read(); //each value needs two registres
     Acc_rawY=Wire.read()<<8|Wire.read();
     Acc_rawZ=Wire.read()<<8|Wire.read();

  Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
     /*---Y---*/
//     Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
     Acceleration_angle[1] = atan((Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
 



   Wire.beginTransmission(0x68);
   Wire.write(0x43); //Gyro data first adress
   Wire.endTransmission(false);
   Wire.requestFrom(0x68,4,true); //Just 4 registers
   
   Gyr_rawX=Wire.read()<<8|Wire.read(); //Once again we shif and sum
   Gyr_rawY=Wire.read()<<8|Wire.read();


      /*---X---*/
   Gyro_angle[0] = Gyr_rawX/131.0; 
   /*---Y---*/
   Gyro_angle[1] = Gyr_rawY/131.0;

   /*---X axis angle---*/
   Total_angle[0] = 0.98 *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle[0];
   /*---Y axis angle---*/
   Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1];


}










פונקציית LCD :
void draw(void) 
{  

    u8g.firstPage();
    do {


            u8g.setFont(u8g_font_6x10);   
        if(state == 2)
            u8g.drawStr( 20, 10, "JOYSTICK MODE ");
        else if(state == 1)
           u8g.drawStr( 10, 10, "accelerometer mode ");
           else if(state == 3)
           u8g.drawStr( 40, 10, "GPS mode ");


  } while( u8g.nextPage() );
 }













פונקציית Keybord :
void readKey()
{
  
  key = digitalRead(D3)*8 + digitalRead(D2)*4 +digitalRead(D1)*2 + digitalRead(D0);
  
  char arrkey[16]={ '1','2','3','A','4','5','6','B','7','8','9','C','*','0','#','D' };
 
//  Serial.println( arrkey[ key ] );  
  //delay(10);

  int t = millis();
  while(digitalRead(DA)==1)    
  {    
     if(millis() - t >500)
      break;
  }
  //FOR SCREEN
   if(arrkey[key] == 'A') state = 1;
   if(arrkey[key] == 'B') state = 2;
   if(arrkey[key] == 'C') state = 3;
} 









פונקציית joystic:
void readJoy()
{
  
  joyX1 = (analogRead(A0)- osJoyX1)/joySensetivity;
  joyY1 = (osJoyY1 - analogRead(A1))/joySensetivity ;
  joyX2 = (osJoyX2 - analogRead(A2)) /joySensetivity ;
  joyY2 = (analogRead(A3) - osJoyY2)/joySensetivity;

}
