/************************ PIN OUTS ************************/
/*
 * BNO055 - I2C PINS : 20(SDA) , 21(SCL)
 * 
 * CYTRON - PWM PINS : 2 , 4 
 *        - DIRECTION PINS : 3 , 5  
 * 
 * LINE FOLLOW PINS - 
 */


#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
// Create the BNO055 object

Adafruit_BNO055 bno = Adafruit_BNO055(55);

//////////// direction pins //////////////////
#define l_dir 6
#define r_dir 4
///////////////// pwm pins///////////////////
#define l_pwm  5
#define r_pwm  3
#define led    13

//////////// line follow pins //////////////////
#define ONE_    7
#define TWO_    8 
#define THREE_  9
#define FOUR_   10 
#define FIVE_   11
#define SIX_    12  
#define SEVEN_  13
#define EIGHT_  14

/*********** PID Variables***********/
float kp = 2;//2 final
float ki = 0.00;//0
float kd = 0.5;//0.5
float hold_angle = (2);  //

float y_kp = 1.6; //0.75;// final
float y_ki = 0.000000;
float y_kd = 5;//70 with dt=1
float y_hold_angle = 0;
bool  yaw_en = 1;
/**************************/
 
////////// yaw angle wrap around variables ///////////////
float current_angle = 0;  // Store the accumulated angle
float previous_angle = 0;
float delta_angle = 0;
////////////////////////////////////////////////////

#define dt 1000/142 //150// ms // 60 working fine with kp = 0.8,kd = 50 , ki = 0.00000001
#define max_control 50
#define R_OFFSET 0//2 //1
#define min_control 3//3
#define buffer_ 0.5

//////////////////////// pitch control variables ////////////////////////////
double time_ , error , rate , P , I , D , control , prev_error , ang , integral;
int value_r, value_l , prev_control;
//////////////////////// yaw control variables //////////////////////////////
float y_ang, y_error, y_P, y_I, y_D, y_rate , y_integral , y_prev_error , y_control ,yaw_val;
/***************************/

void setup() {
  // Initialize serial communication
  Serial.begin(19200);
  bno_setup();
  //////////// pin config /////////////
  pinMode(l_dir , OUTPUT);
  pinMode(r_dir , OUTPUT);
  pinMode(l_pwm , OUTPUT);
  pinMode(r_pwm , OUTPUT);
  pinMode(led   , OUTPUT);
  
  pinMode(ONE_   , INPUT);
  pinMode(TWO_   , INPUT);
  pinMode(THREE_ , INPUT);
  pinMode(FOUR_  , INPUT);
  pinMode(FIVE_  , INPUT);  
  pinMode(SIX_   , INPUT);
  pinMode(SEVEN_ , INPUT);
  pinMode(EIGHT_ , INPUT);
     
  digitalWrite(l_dir, 0);
  digitalWrite(r_dir, 0);
  digitalWrite(led, 0);
  //   motor_left(1,3);//min 10 // max 100
  //   motor_right(1,3);//min 10 //max 100
  //   while (1);
  // Start reading angles
  sensors_event_t event;
  bno.getEvent(&event);
  previous_angle = event.orientation.x;  // Initialize previous angle with current reading  
}

void loop() {

  while (1) // make this while 1 to check CG
  {
    sensors_event_t event;

    bno.getEvent(&event);

    Serial.print("\tPitch: ");
    Serial.print(event.orientation.y);
    Serial.print("\tyaw: ");
    Serial.print(get_yaw());
    Serial.println();
    delay(100);
  }
  while (1)
  {
    control_pid(hold_angle, 0, 1000);
  }
}



void bno_setup()
{
  Serial.println("Orientation Sensor Test"); Serial.println("");
  if (!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
}
void motor_left(int dir, int speed_)
{
  if (dir == 1)
  {
    digitalWrite(l_dir, 1);
  }
  else
  {
    digitalWrite(l_dir, 0);
  }
  analogWrite(l_pwm, speed_);
}
void motor_right(int dir, int speed_)
{
  if (dir == 1)
  {
    digitalWrite(r_dir, 1);
  }
  else
  {
    digitalWrite(r_dir, 0);
  }
  analogWrite(r_pwm, speed_);
}

///////////////////////////////// adding yaw control ////////////////////////////////////
float yaw_control(float desired_yaw)
{

  static int y_min = 8 , y_max = 30, yc;
  /******** PID Algorithm ********/
  float y_t = get_yaw();
  y_error = desired_yaw - (y_t);
  //////////// limiting error ////////////////
  if (fabs(y_error) > 10)
  {
    y_error = (fabs(y_error) / y_error) * 10;
  }
  else if (fabs(y_error) < 2)
  {
    y_error = 0;
    y_integral = 0;
  }
  y_rate = (y_error - y_prev_error) / dt;
  y_integral += (y_error * dt);
  y_P = y_kp * y_error;
  y_I = y_ki * y_integral;
  y_D = y_kd * y_rate;
  if (fabs(y_I) > 5)
  {
    y_I = (fabs(y_I) / y_I) * 5;
  }
  yc = y_P + y_I + y_D;
  if (fabs(yc) > y_max)
  {
    yc = (fabs(yc) / yc) * y_max;
  }
  else if (fabs(yc) < y_min)
  {
    yc = (fabs(yc) / yc) * y_min;
  }
  y_prev_error = y_error;
  return yc;
}
/////////////////////////wrap around yaw angle +ve and -ve /////////////////////////////////
float get_yaw(void)
{
  sensors_event_t event;
  bno.getEvent(&event);

  float new_angle = event.orientation.x;  // Read yaw (heading angle, 0-360 degrees)

  // Calculate the difference (delta) between new and previous angle
  delta_angle = new_angle - previous_angle;

  // Handle wrap-around from 360° to 0°
  if (delta_angle > 180) {
    delta_angle -= 360;  // Positive wrap-around
  } else if (delta_angle < -180) {
    delta_angle += 360;  // Negative wrap-around
  }
  
  // Accumulate the total angle
  current_angle += delta_angle;

  // Update previous angle for the next loop iteration
  previous_angle = new_angle;

  return current_angle;
}



void control_pid (float pitch , float yaw , int loop_)
{
  sensors_event_t event;
  int i = 0;
  while(1)
  {
    if ((millis() - time_) >= dt)
    {
      bno.getEvent(&event);
      ang = event.orientation.y;
      /******** PID Algorithm ********/
      error = pitch - ang;
      //////////// limiting error////////////////
      if (fabs(error) > 40)
      {
        error = (fabs(error) / error) * 40;
      }
      else if (fabs(error) < buffer_)
      {
        error = 0;
        integral = 0;
      }
      rate = (error - prev_error) / dt;
      integral += (error * dt);
      P = kp * error;
      I = ki * integral;
      D = kd * rate;
      if (fabs(I) > 20)
      {
        I = (fabs(I) / I) * 20;
      }
      control = P + I + D;
      if (fabs(control) > max_control)
      {
        control = (fabs(control) / control) * max_control;
      }
      else if ((fabs(control) < min_control) && (error != 0) && (kp != 0))
      {
        control = (fabs(control) / control) * min_control;
      }
      prev_error = error;
      prev_control = control;
      time_ = millis();
      if (yaw_en)
      {
        //control=0;
        yaw_val = yaw_control(yaw);
        value_l = fabs(control - yaw_val ) / (control - yaw_val);
        value_r = fabs(control + yaw_val ) / (control + yaw_val );
        motor_left((-1)*value_l, fabs(control - yaw_val ) );
        motor_right((-1)*value_r, fabs(control + yaw_val ));
      }
      else
      {
        value_l = fabs(control) / control;
        value_r = fabs(control) / control;
        motor_left((-1)*value_l, fabs(control));
        motor_right((-1)*value_r, fabs(control));
      }
      Serial.print(value_l); // left
      Serial.print(",");  
      Serial.println(control); 
      i++;
      if(i>=loop_)
      {
       break;
      }
    }
  }
}
