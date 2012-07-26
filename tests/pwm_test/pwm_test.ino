/*   

basic pwm test
25/7/12

   a basic rig to test the motor controller.
   uses a potentiometer to read pwm value,and a toggle switch to change direction.
   here a single pwm value is send to both motors, so connect PWM1 to both channels of the motorcontroller.
   This can be used to test the motor controller, for a given pwm value the motor controller should supply the same vlotage
   to both the channels.

*/

#define PWM1	3 //pin to send pwm value to motor controller.
#define DIR1    2 //pin to send direction to motor controller
#define BRK1	4

#define POT1	A4 // a potentiometer to getpwm value (0-255)
#define TOG1    A5 //a togle swith to change direction

void setup() {

  pinMode(PWM1, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(BRK1, OUTPUT);

  pinMode(POT1, INPUT);
  pinMode(TOG1, INPUT);

  Serial.begin(9600);
}

void loop() {
int pwmVal=analogRead(POT1)*(255.0/1024);// this will convert a value in the range 0-1024 received from the potentiometr \
                                            to a value in the range 0-255.
int dirVal=digitalRead(TOG1);
Serial.print(pwmVal);
Serial.print("  ");
Serial.println(dirVal);

if(pwmVal==0){
	digitalWrite(BRK1,HIGH);//apply brake if pwmVal==0
}
else{
	digitalWrite(BRK1,LOW);//apply brake if pwmVal==0
	digitalWrite(DIR1,dirVal);
	analogWrite(PWM1,pwmVal);
}

}
