/*
calibertion of accelerometer to find min/max values
26/7/12

this is used to find the min and max of volatge values given by the accceloromter
along each of its axises this code assumes 3 axises X,Y,z. 

PROCESS:
--------
1)with this code running, tilt the acceloromter in all intended orientations that need to be measured
2) see the results in the arduiono ide serial monitor
3) after some time the values will stop changing, these are the min/max values for each axis
4) these values can be used in a MAP routine to get the meaured tilt in degrees.

map(inMin,inMax,outMin,outMax,inNow){
/*
inMin, min value for the axis
inMax, max value for the asis
outMin, min value of tilt, 0 degree
outMax, max value for tile, 180 or 360 degree
/*

return ((inMax-inMin)/(outMax-outMin)*inNow);

}




this code and method was taken from some book on arduino found online!

*/
const unsigned int X_AXIS_PIN = 0;
const unsigned int Y_AXIS_PIN = 1;
const unsigned int Z_AXIS_PIN = 2;

const unsigned int BAUD_RATE = 9600;

int min_x, min_y, min_z;
int max_x, max_y, max_z;

void setup() {
  Serial.begin(BAUD_RATE);
  min_x = min_y = min_z = 1000;
  max_x = max_y = max_z = -1000;
}

void loop() {
  const int x = analogRead(X_AXIS_PIN);
  const int y = analogRead(Y_AXIS_PIN);
  const int z = analogRead(Z_AXIS_PIN);

  min_x = min(x, min_x); max_x = max(x, max_x);
  min_y = min(y, min_y); max_y = max(y, max_y);
  min_z = min(z, min_z); max_z = max(z, max_z);
  
  Serial.print("x(");
  Serial.print(min_x);
  Serial.print("/");
  Serial.print(max_x);
  Serial.print("), y(");
  Serial.print(min_y);
  Serial.print("/");
  Serial.print(max_y);
  Serial.print("), z(");
  Serial.print(min_z);
  Serial.print("/");
  Serial.print(max_z);
  Serial.println(")");
}


