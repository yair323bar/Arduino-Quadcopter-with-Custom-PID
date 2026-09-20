#include <Wire.h>

//ultrasony typs
int distance,duration; //משך הזמן שלוקח לשידור לחזור 
int Trig=5; //שידור
int Echo=6; //קבלת שידור

////////////////////////GY-65 liberaries_and types/////////////////////////
#include <Adafruit_BMP085.h>

Adafruit_BMP085 bmp;

int Temperature,Altitude ;

//__________________________________________________________


/////////////////////////gps liberaries_and types///////////////////////////
#include <SoftwareSerial.h>
#include <TinyGPS.h>

TinyGPS NEO6M;
const int RXpin = 3 ;// Connect with TX pin in GPS
const int TXpin = 4 ;// Connect with RX pin in GPS
SoftwareSerial Genotronex(RXpin,TXpin);
unsigned int buadRateGPS= 9600 ;
unsigned long buadRateUART= 115200 ;
const unsigned int DelayGPStime = 1000 ;




float  fLat, fLon;
unsigned long age;
int year;
byte month, day, hour, minute, second, hundredths;
//__________________________________________________________
int arr[11];

void setup() 
{
  Wire.begin(8); // join i2c bus (address optional for master)
  Wire.onRequest(requestEvent);
//ultrasony conect
  pinMode(Trig,OUTPUT);
  pinMode(Echo,INPUT);
  //GY-65
  bmp.begin ();
  Serial.begin(9600);
}

byte x = 0;

void loop()
{
  ultrasony();
  GPS();
  GY65();







  arr[0]=fLat;
  arr[1]=fLon;
  arr[2]=year;
  arr[3]=month;
  arr[4]=day;
  arr[5]=hour;
  arr[6]=minute;
  arr[7]=second;
  arr[8]=distance;
  arr[9]=Altitude;
  arr[10]=Temperature;
}

void requestEvent()
{
  for(int i=0;i<11;i++)
  { 
      Wire.write(arr[i]>>8 & 0x0F);
      Wire.write(arr[i] & 0x0F);                 
  }
}
 











פונקציות עבד (Void):
פונקציית GPS:
void GPS()
{
  bool newData = false;
 for (unsigned long start = millis(); millis() - start <
DelayGPStime;)
 {
 while (Genotronex.available())
 {
 char c = Genotronex.read();
 // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
 if (NEO6M.encode(c)) // Did a new valid sentence come in?
 newData = true;
 }
 }
 if (newData)
 {

 NEO6M.f_get_position(&fLat, &fLon, &age);//flat= קו רוחב || flon= קו אורך

 NEO6M.crack_datetime(&year, &month, &day,&hour, &minute, &second,&hundredths);

 }
 }







void GY65()
{

//read Temperature
  Temperature = bmp.readTemperature ();
  
//read Altitude  
  Altitude = bmp.readAltitude ( 101500 );



    Serial.print ( "Temperature =" );
    Serial.print (Temperature);
    Serial.println ( "C *" );

    Serial.print ( "real Altitude =" );
    Serial.print (Altitude );
    Serial.println ( "m" );
    Serial.println ();
}












void ultrasony()
{
  digitalWrite(Trig,LOW);
  
  delayMicroseconds(2);
  
  digitalWrite(Trig,HIGH);
  
  delayMicroseconds(10);
  
  digitalWrite(Trig,LOW);
  
  duration= pulseIn(Echo,HIGH);
  
  distance = duration* 0.01715 ;   
}
