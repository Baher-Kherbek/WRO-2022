
/*----------------------This is the Code for the Microcontroller of the main Robot which utilizes the robot operating system------------------------
 * Created by the No_Mercy_NCD team
 * 
 * 
 */


#include <ros.h>
#include <geometry_msgs/Quaternion.h>
#include <Servo.h>

//first dc:
int en11 = A0;
int en12 = A1;
int motor1L = 2;
int motor1R = 3;

//second dc:
int en21 = A2 ;
int en22 = A3;
int motor2R = 4;
int motor2L = 5;

//fourth dc:
int en41 = A4 ;
int en42 = A5 ;
int motor4R = 6;
int motor4L = 7;

//Third dc:
int en31 = A6 ;
int en32 = A7;
int motor3L = 8;
int motor3R = 9;

#define echoPin  A8
#define trigPin  A9

long duration;
int distance;

Servo slider;
Servo base_;
Servo link;
Servo wrest;

bool mask = true;
ros::NodeHandle  nh;

void check_mask()
{
  base_.write(0);
  link.attach(13);
  while (digitalRead(26))
  {
    link.write(180);
  }
  link.write(96);
  delay(3000);
  link.write(92);
  delay(3500);
  link.detach();
  link.attach(13);
}
void give_mask()
{
  bool ch = false;
  bool ch2 = false;
  int done = 1;
  base_.write(0);
  if (!digitalRead(22)) {
    if (done == 1) {
      while (digitalRead(24)) {
        slider.attach(12);
        slider.write(87);
      }
      done ++;
    }
    else if (done > 1) {
      slider.detach();
      link.attach(13);
      if (!ch2)
      {
        while (digitalRead(26))
        {
          link.write(180);
        }
        link.write(96);
        base_.attach(11);
        slider.attach(12);
        while (digitalRead(24))
        {
          slider.write(87);
        }
        ch2 = true;
      }
    }
  }
  else if (!digitalRead(24)) {

    slider.attach(12);
    if (!ch)
    {
      wrest.attach(10);
      int time = millis();
      while (millis() - time < 3000)
      {
        wrest.write(180);
      }
      wrest.detach();
      ch = true;
    }
    else {
      delay(2000);
      int  tiime = millis();
      while (millis() - tiime < 1500) {
        wrest.write(0);
      }
      wrest.write(90);
      while (digitalRead(22)) {
        slider.write(80);
      }
    }
  }
}
void messageCb( const geometry_msgs::Quaternion& sp) {
  digitalWrite(en11, HIGH);
  digitalWrite(en12, HIGH);
  digitalWrite(en21, HIGH);
  digitalWrite(en22, HIGH);
  digitalWrite(en31, HIGH);
  digitalWrite(en32, HIGH);
  digitalWrite(en41, HIGH);
  digitalWrite(en42, HIGH);

  if (sp.x < 0 & sp.y < 0 & sp.z < 0 & sp.w < 0) {
    analogWrite(motor1R, 0);
    analogWrite(motor1L, abs(sp.x));
    analogWrite(motor2R, 0);
    analogWrite(motor2L, abs(sp.y));
    analogWrite(motor3R, 0);
    analogWrite(motor3L, abs(sp.z));
    analogWrite(motor4R, 0);
    analogWrite(motor4L, abs(sp.w));
  }
  else if (sp.x == 0 & sp.y == 0 & sp.z != 0 & sp.w != 0) {
    analogWrite(motor1L, 0);
    analogWrite(motor1R, 0);
    analogWrite(motor2R, 0);
    analogWrite(motor2L, sp.z);
    analogWrite(motor3R, 0);
    analogWrite(motor3L, 0);
    analogWrite(motor4R, sp.w);
    analogWrite(motor4L, 0);
  }
  else if (sp.x == 0 && sp.y == 0 && sp.z == 0 && sp.w == 0)
  {
    analogWrite(motor1R, 0);
    analogWrite(motor1L, 0);
    analogWrite(motor2R, 0);
    analogWrite(motor2L, 0);
    analogWrite(motor3R, 0);
    analogWrite(motor3L, 0);
    analogWrite(motor4R, 0);
    analogWrite(motor4L, 0);
  }
}

void ultra()
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;
}

ros::Subscriber<geometry_msgs::Quaternion> sub("chaTTer", messageCb );

void setup() {
  //ultra
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  //arm
  wrest.detach();
  pinMode(24, INPUT_PULLUP);
  pinMode(22, INPUT_PULLUP);
  pinMode(26, INPUT_PULLUP);
  base_.attach(11);
  base_.write(0);
  //navigate
  pinMode(motor1R, OUTPUT);
  pinMode(motor1L, OUTPUT);
  pinMode(motor2R, OUTPUT);
  pinMode(motor2L, OUTPUT);
  pinMode(motor3R, OUTPUT);
  pinMode(motor3L, OUTPUT);
  pinMode(motor4R, OUTPUT);
  pinMode(motor4L, OUTPUT);

  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(A6, OUTPUT);
  pinMode(A7, OUTPUT);

  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A4, OUTPUT);
  pinMode(A5, OUTPUT);
  //ros

  nh.initNode();
  nh.subscribe(sub);

}
void loop()
{
  ultra();
  if (distance <= 45 && distance >= 10)
  {
    check_mask();
    if (not_mask)
    {
      give_mask();
    }
  }
  nh.spinOnce();
  delay(10);
}
