/*-------------------------This is the algorithm used for line following---------------------------
 * 
 * 
 * Created by the No_Mercy_NCD Team
 */

#include <Servo.h>

float kp = 0.2;
float ki = 0;
float kd = 1.2;
float error = 0;
float lasterror = 0;
  int cont = 0;
Servo Left, Right;
void setup() {
  Serial.begin(9600);
  Left.attach(6);
  Right.attach(5);
}
int PID(){
  while (true){
    float val = analogRead(A0);
    float val2 = analogRead(A7);
    sum += error;
    lasterror = error;
    error = val - val2;
    float d = error - lasterror ;
    float co = error * kp + d*kd ;
    //co = co % 90;
    co = constrain(co, -90 , 90);

  cont ++ ;
  Serial.print("s1=" );
  Serial.println(val );
  Serial.print("s2=" );
  Serial.println(val2 );
  Left.write(90 + 45 + co/2);
  Right.write(45 + co/2);
    if (val >900){
      return 1;
    }
    else if (val2 >900){
      return 2;
    }
  }
}
void turn(int b){
  if (b == 2){
    while (true){
      float val = analogRead(A0);
      float val2 = analogRead(A7);
      Left.write(180);
      Right.write(90);
      if (val > 300){
      break;
      }
    }
  }
  if (b == 1){
   while (true){
      float val = analogRead(A0);
      float val2 = analogRead(A7);
      Left.write(90);
      Right.write(0);
      if (val2 > 600){
      break;
      }
   }
  }
}
void loop() {
  int b = PID();
  turn(b);
}
