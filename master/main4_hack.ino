


/*

main
*/


#include <LiquidCrystal.h>
#include <math.h> 
#include <PID_v1.h>


LiquidCrystal lcd(13,12,11, 10, 9, 8);
// initialize the library with the numbers of the interface pins


//-------------switch maps
const int mode_switch=A5;
const int menu_pot=A3;
const int delta_pot=A4;
const int param_up_switch=6;
const int param_down_switch=7;


//--params


double live_params[]={
2,//kp
5,//ki
1,//kd
0,//base angle
.80,//cfw
1,//gyd
1,//lfb
0,//nlf
0//ser
};

char* live_params_label[]={
"PRO  ",
"INT  ",
"DIF  ",
"ANG  ",
"CFW  ",
"GYD  ",
"LFB  ",
"NLF  ",
"SER"
};

int param_c=9;
int delta_c=12;
float delta[]={
.01,
.05,
.1,
.10,
.25,
1.0,
10,
50,
100,
1000,
10000
};


//----------
const int throttle_l_pin=6;
const int throttle_r_pin=3;
const int brake_l_pin=7;
const int brake_r_pin=4;
const int dir_l_pin=5;
const int dir_r_pin=2;


//----------

const int XMAX = 418;
const int XMIN = 157;
const int YMAX = 334;
const int YMIN = 222;
const int ZMAX = 289;
const int ZMIN = 212;

const float radToDeg = 57.2957795;

const int controlChar = 119;

//const int input_X = 0; 
const int input_Y = A0;
const int input_Z = A1;


int xVal, yVal, zVal;
 
float xAng, yAng, zAng;
  
float x,y,z;

int charBuffer=0;

// Variables for the moving average filter

const int filterSize = 1; // Define the filter's width here. 

float filter_X[filterSize];  // Declare it.
float filter_Y[filterSize];
float filter_Z[filterSize];

int filterUpdatePos = 0;  // This variable indicates which array position to update.

float avg_x = 0;
float avg_y = 0;
float avg_z = 0;




//---------------functions
float floatmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void updateGlobals()
{
  //xVal = analogRead(input_X);
  yVal = analogRead(input_Y);
  zVal = analogRead(input_Z);
  
  //xAng = floatmap(xVal, XMIN, XMAX, -90, 90);
  yAng = floatmap(yVal, YMIN, YMAX, -90, 90);
  zAng = floatmap(zVal, ZMIN, ZMAX, -90, 90);
  
  
  x =yAng ;
  //radToDeg * (atan2(yAng, -zAng) + PI);
  
  //y = radToDeg * (atan2(xAng, -zAng) + PI);
  
 
  
  if(x>180)
      x= floatmap(x, 180,360,-180,0);  
      
      
      
  updateFilter();
    
}


void updateFilter()
{
   // First, we get the values from the analog in pins and then store them in the current position.
   
   filter_X[filterUpdatePos] = x;
  // filter_Y[filterUpdatePos] = y;
 
   // Next, update the pointer. 
   if (filterUpdatePos >= live_params[6])
   {
     filterUpdatePos = 0;  
   }
   else  
   {
      filterUpdatePos++;
 
    }
  
}



void updateAverage()
{
  float xTemp;
  //float yTemp;

  xTemp = 0;
  
  for (int i=0; i<live_params[6]; i++)
  {
    xTemp += filter_X[i];     
  }
 
  
  avg_x = xTemp/live_params[6];
 
  
  
}



int PIN_GYRO_X=A2;
int stationary_val=0;

float CalGyro() {
    float val = 0.0;
    digitalWrite(13, HIGH);
    delay(500);
    for (int i = 0; i < 25; i++) {
        val += analogRead(PIN_GYRO_X);
        delay(10);
    }
 
    val /= 25.0;
 
//   Serial.print("GyroX Stationary=");
  //  Serial.println(val);
    digitalWrite(13, LOW);
 delay(1000);
    return val;
}


int menu_index=0;
int delta_index=0;


boolean  debounced(int pin){
int val1=0;

val1=digitalRead(pin);
delay(80);
if (digitalRead(pin)==HIGH && val1==HIGH)
return(LOW);
else if(digitalRead(pin)==LOW && val1==LOW)
return(HIGH);

}




int inByte;
float VxGyro=0;
float fusion=0;
float AccAngle=0;
float current=0,prev=0,dt=0;
float cur_angle=0;
boolean isFirstTime=true;
float currAngle=0,prevAngle=0;
float degX=0;

float cfw=0.90;
float gyr_damp=1;
float EstimateAngle() {
cfw=live_params[4];
gyr_damp=live_params[5];
current=micros();

dt=(current-prev)/1e6f;
prev=current;
  
  updateGlobals();
  updateAverage();
  
    AccAngle =(x+live_params[7]);
    //Serial.println(AccAngle);
    if (isFirstTime) {
        currAngle = AccAngle;
        isFirstTime = false;
    } else {
        VxGyro = analogRead(PIN_GYRO_X);
//Serial.print(VxGyro);
//Serial.print("|");
  degX = (float) (VxGyro - stationary_val)/9.0* (float) dt*(float)(180/PI);
       //Serial.println(degX);
        currAngle = cfw * (prevAngle -degX) + (1-cfw)* AccAngle;
    }
 
 //Serial.println(currAngle);
    prevAngle = currAngle;
 
    //Debugging code omitted here, see attached source for the full code listings.
 
    return currAngle;
}



double throttle,output;
double fusion_angle=0,f_a=0;
PID myPID(&f_a, &throttle, &live_params[3],2,5,1, DIRECT);


void setup(){
  Serial.begin(9600);
  lcd.begin(16, 2);
pinMode(7,INPUT);
pinMode(6,INPUT);
//pinMode(6, OUTPUT);
pinMode(3, OUTPUT);
pinMode(2,OUTPUT);
pinMode(4,OUTPUT);

//------------------
  for (int i=0; i< filterSize; i++)
  {
   // Serial.print("Updating Position ");
    //Serial.println(filterUpdatePos);
    updateFilter();
  }  
  stationary_val=270;
  
  //establishContact();  
  
 delay(1000); 

 myPID.SetMode(AUTOMATIC);
myPID.SetSampleTime(50); 
myPID.SetOutputLimits(-255, 255); 

}


boolean was_mode=true;

void loop(){
//Serial.println("sdfscd");
  if(digitalRead(mode_switch)==HIGH){
    digitalWrite(3,0);
  
  if (was_mode==false){
lcd.clear();
was_mode=true;
}

    menu_index=(param_c/1024.0)*analogRead(menu_pot);
    delta_index=(delta_c/1024.0)*analogRead(delta_pot);
    
      if (debounced(param_up_switch)==HIGH )
       live_params[menu_index]+=delta[delta_index];

      if(debounced(param_down_switch)==HIGH)
        live_params[menu_index]-=delta[delta_index];
    
    if(menu_index<=param_c-2 && delta_index<=delta_c){
    lcd.setCursor(0,0);
    lcd.write(live_params_label[menu_index]);
    lcd.setCursor(6,0);
    lcd.write("del:");
    lcd.setCursor(10,0);
    lcd.print(delta[delta_index]);
    
    lcd.setCursor(0,1);
    lcd.print(live_params[menu_index]);
    
    }
    else if(menu_index==8){
     
//was_mode=false;
lcd.setCursor(0,0);

          
   fusion_angle=-EstimateAngle()-live_params[7];
lcd.setCursor(0,0);
//Serial.println(fusion_angle);
lcd.print(fusion_angle);
    
    }
    
    myPID.SetTunings(live_params[0],live_params[1],live_params[2]); 
   
    }
    
  
  
  else{
  //Serial.println("sdf");

if (was_mode==true){
lcd.clear();
was_mode=false;
}
  fusion_angle=-EstimateAngle()-live_params[7];
  f_a=fusion_angle;
  
  

//lcd.clear();


myPID.Compute();


lcd.setCursor(0,0);
//Serial.println(fusion_angle);
lcd.print(fusion_angle);
lcd.setCursor(0,1);
lcd.print(throttle);


if(throttle<0){
//digitalWrite(dir_l_pin,LOW);
digitalWrite(2,LOW);
//Serial.print("forw");
//Serial.print ("|");
//Serial.println(abs(throttle));
}
else{
//digitalWrite(dir_l_pin,HIGH);
digitalWrite(2,HIGH);
//Serial.print("back");
//Serial.print ("|");
//Serial.println(throttle);
}

//analogWrite(throttle_l_pin,throttle);

analogWrite(throttle_r_pin,abs(throttle));

}
  

}





/*
27th September 2011
Getting ADXL335 data filtered and onto the PC via python.

// Values calibrated from *my* accelerometer. Calibrate yours today!
//  

*/






