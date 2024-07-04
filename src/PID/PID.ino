#include <ArduinoQueue.h>
#include <NewPing.h>
#include<math.h>

#define TRIGGER_PIN_L1  36
#define ECHO_PIN_L1  37

#define TRIGGER_PIN_L2  38
#define ECHO_PIN_L2  39

#define TRIGGER_PIN_R1  34
#define ECHO_PIN_R1  35

#define TRIGGER_PIN_R2  32
#define ECHO_PIN_R2  33

#define MAX_DISTANCE 300

#define led_fr 51
#define led_br 50
#define led_fl 49
#define led_bl 48

#define Switch 21 

NewPing sonar_L1(TRIGGER_PIN_L1, ECHO_PIN_L1, MAX_DISTANCE);
NewPing sonar_L2(TRIGGER_PIN_L2, ECHO_PIN_L2, MAX_DISTANCE);
NewPing sonar_R1(TRIGGER_PIN_R1, ECHO_PIN_R1, MAX_DISTANCE);
NewPing sonar_R2(TRIGGER_PIN_R2, ECHO_PIN_R2, MAX_DISTANCE);

double duration_F = 0, duration_L1 = 0, duration_L2 = 0, duration_R1 = 0, duration_R2 = 0, duration_B = 0;
double distance = 0, real_distance = 0 , Real_dis_L = 0, Real_dis_R = 0, dis_L1 = 0, dis_L2 = 0, dis_R1 = 0, dis_R2 = 0, dis_F = 0, dis_B =0;
volatile int Switch_State = 0;
double highest_angle = 0.27;
double pos_distance;
double jbes = 0;
double position_angle = 0;
double angle = 0, Angle = 0, Aangle = 0;
 
int Filter_Size = 10;
double temp = 0;
double mean_bl = 0;
ArduinoQueue <double> Means_BL;
double mean_fl = 0;
ArduinoQueue <double> Means_FL;
double mean_br = 0;
ArduinoQueue <double> Means_BR;
double mean_fr = 0;
ArduinoQueue <double> Means_FR;

double br , fl, bl , fr ;


int in1 = 3;
int in2 = 4;
int enA = 2;
int in3 = 6;
int in4 = 7;
int enB = 5;

int pwm = 100;

void switch_handle()
{
  Switch_State = !Switch_State;
  while(!Switch_State)
  {
    runLeftMotorForward(0);
    runRightMotorForward(0);
    turn_leds(0,0,0,0);
  }
}

double Filter(double read, double &last_mean, ArduinoQueue <double> &Means)
{
  if (Means.isEmpty())
  { 
    last_mean = read;
    for (int i = 0; i < Filter_Size; i++)
    {
      Means.enqueue(read);
    }
  }
  else
  {
    temp = Means.dequeue();
    last_mean -= (double(temp / Filter_Size));
    last_mean += (double(read / Filter_Size));
    Means.enqueue(read);
  }
  return last_mean;
}

double get_distance_L1 ()
{
  duration_L1 = sonar_L1.ping();
  fl = ((duration_L1 / 2) * 0.0343) - 4 ;
  
  mean_fl = Filter(fl, mean_fl, Means_FL);
  return mean_fl ;
}

double get_distance_L2()
{
  duration_L2 = sonar_L2.ping();
  bl = ((duration_L2 / 2) * 0.0343) - 4 ;
  mean_bl = Filter(bl, mean_bl, Means_BL);
  return mean_bl;
}

double get_distance_R1 ()
{
  duration_R1 = sonar_R1.ping();
  fr = ((duration_R1 / 2) * 0.0343)-4;
  mean_fr = Filter(fr, mean_fr, Means_FR);
  return mean_fr;
}

double get_distance_R2()
{
  duration_R2 = sonar_R2.ping();
  br = ((duration_R2 / 2) * 0.0343) - 4;
  mean_br = Filter(br, mean_br, Means_BR);
  
  return mean_br ;
}

double angle_cal (double dis_1 , double dis_2)
{
  Angle =  atan ((dis_1 - dis_2 ) / 7.4 );
  if (Angle > highest_angle)
  {
    Angle =  highest_angle;
  }
  if (Angle < ((-1)*highest_angle))
  {
    Angle = (-1) * highest_angle;
  }
  return Angle;
}

// PID constants
double Kp = 1.0;  // Proportional gain
double Ki = -0.05;  // Integral gain
double Kd = 0.0;  // Derivative gain

// Variables for PID control
double previous_error = 0;
double integral = 0;
double derivative = 0;

// Target angle (desired angle)
double target_angle = 0;

double angle_Left(double L1, double L2) {
  angle = angle_cal(L1, L2);

  Angle = (angle * 180) / 3.14;

  // Calculate error (difference between target and current angle)
  double error = Angle;

  // Calculate integral term
  integral += error;

  // Calculate derivative term
  derivative = error - previous_error;
  previous_error = error;

  // Calculate PID output
  double PID_output = Kp * error + Ki * integral + Kd * derivative;

  // Adjust motor speed based on PID output
  int left_motor_pwm = pwm + PID_output;
  int right_motor_pwm = pwm - PID_output;

  // Ensure PWM values are within valid range (0-255)
  left_motor_pwm = constrain(left_motor_pwm, 0, 255);
  right_motor_pwm = constrain(right_motor_pwm, 0, 255);

  // Apply PWM values to motors
  runLeftMotorForward(left_motor_pwm);
  runRightMotorForward(right_motor_pwm);

  return Angle; // Return the current angle (optional)
}

void sensor() {
 
  dis_L1 = get_distance_L1();
  delay(10);
  dis_R2 = get_distance_R2();

  delay(10);

  dis_R1 = get_distance_R1();
  delay(10);
  dis_L2 = get_distance_L2();
   
  delay(10);
  Real_dis_L = Distance(fl, bl);
  Real_dis_R = Distance(fr, br);
}


void runLeftMotorForward (int pwm)
{
  digitalWrite (in1,HIGH);
  digitalWrite (in2,LOW);
  analogWrite (enA,pwm);
}

void runRightMotorForward (int pwm)
{
  digitalWrite (in4,HIGH);
  digitalWrite (in3,LOW);
  analogWrite (enB,pwm);
}
void runLeftMotorBackward (int pwm)
{
  digitalWrite (in2,HIGH);
  digitalWrite (in1,LOW);
  analogWrite (enA,pwm);
}

void runRightMotorBackward (int pwm)
{
  digitalWrite (in3,HIGH);
  digitalWrite (in4,LOW);
  analogWrite (enB,pwm);
}

void turn_leds(bool fr_led,bool fl_led,bool br_led,bool bl_led)
{
  digitalWrite(led_fr,fr_led);
  digitalWrite(led_fl,fl_led);
  digitalWrite(led_br,br_led);
  digitalWrite(led_bl,bl_led);
}

void setup() {
  pinMode (in1, OUTPUT);
  pinMode (in2, OUTPUT);
  pinMode (enA, OUTPUT);
  pinMode (in3, OUTPUT);
  pinMode (in4, OUTPUT);
  pinMode (enB, OUTPUT);
  pinMode (led_fr, OUTPUT);
  pinMode (led_fl, OUTPUT);
  pinMode (led_br, OUTPUT);
  pinMode (led_bl, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(Switch),switch_handle,CHANGE);
  while(!Switch_State);
  
}


void loop() {
  sensor();
  angle_Left();
}
