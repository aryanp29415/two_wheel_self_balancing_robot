#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
// Create the BNO055 object

Adafruit_BNO055 bno = Adafruit_BNO055(55);

/*********** PID Variables***********/
float kp = 5;//2 final
float ki = 0.005;//0
float kd = 5;//0.5
float hold_angle = (1.7);  //
  
 float y_kp = 1.6; //0.75;// final
float y_ki = 0.000000;
float y_kd = 5;//70 with dt=1
float y_hold_angle = 0;
bool yaw_en = 1;
/**************************/
//////////// direction pins //////////////////
const int l_dir = 6, r_dir = 4 ;
/////////// pwm pins///////////////////
const int l_pwm = 5, r_pwm = 3 , led= 13;
float yaw_val;
#define dt 1000/142 //150// ms // 60 working fine with kp = 0.8,kd = 50 , ki = 0.00000001
#define max_control 150
#define R_OFFSET 0//2 //1
#define min_control 5//3
#define buffer_ 0.5

//////////////////////// pitch control variables ////////////////////////////
double time_ , error , rate , P , I , D ,control , prev_error , ang , integral;
int value_r, value_l , prev_control;
//////////////////////// yaw control variables //////////////////////////////
float y_ang, y_error, y_P, y_I, y_D, y_rate , y_integral , y_prev_error ,y_control;
/***************************/
void setup() {
  // Initialize serial communication
  Serial.begin(19200);
  bno_setup();
  //////////// pin config /////////////
  pinMode(l_dir, OUTPUT);
  pinMode(r_dir, OUTPUT);
  pinMode(l_pwm, OUTPUT);
  pinMode(r_pwm, OUTPUT);
  pinMode(led , OUTPUT);
  digitalWrite(l_dir, 0);
  digitalWrite(r_dir, 0);
  digitalWrite(led, 0);
   motor_left(1,0);//min 10 // max 100
   motor_right(1,0);//min 10 //max 100
   while (1);
}
void loop() {
 
  while (0) // make this while 1 to check CG
  {
    sensors_event_t event;
    bno.getEvent(&event);
    Serial.print("\tPitch: ");
    Serial.print(event.orientation.y);
    Serial.print("\tyaw: ");
    Serial.print(convertYaw(event.orientation.x));
    Serial.println();
    delay(100);
  }
  while (1)
  {
    control_pid(hold_angle,y_hold_angle,1000);
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
float yaw_control(float y_t , float desired_yaw)
{
  static int y_min = 3 , y_max = 18,yc;
  /******** PID Algorithm ********/
  y_error = desired_yaw -convertYaw(y_t);
  //////////// limiting error ////////////////
  if (fabs(y_error) > 10)
  {
    y_error = (fabs(y_error) /y_error) * 10;
  }
  else if (fabs(y_error) < 2)
  {
    y_error = 0;
    y_integral = 0;
  }
  y_rate = (y_error - y_prev_error) /dt;
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
///////////////////////// mapping -180 to 180 /////////////////////////////////
float convertYaw(float yaw)
{
  if (yaw > 180.0)
  {
    yaw -= 360.0;
  }
  return yaw;
}



void control_pid (float pitch , float yaw , int loop_)
{
   sensors_event_t event;
   
  for(int i =0; i<loop_ ; i++)
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
        error = (fabs(error) / error) *40;
      }
      else if (fabs(error) < buffer_)
      {
        error = 0;
        integral = 0;
      }
//      if ((fabs(error) > buffer_))
//      {
//        if((fabs(error) >5))
//        {
//        error = error+(fabs(error) /error) *18;
//        }
//        else
//        {
//        error = error+(fabs(error) /error) *6;
//        }     
//      }
      rate = (error - prev_error) /dt;
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
        control = (fabs(control) /control) * max_control;
      }
      else if ((fabs(control) <min_control) && (error != 0) &&(kp != 0))
      {
       control = (fabs(control) /control) * min_control;
      }
      prev_error = error;
      prev_control = control;
      time_ = millis();
      if (yaw_en)
      {
        //control=0;
        yaw_val =yaw_control((event.orientation.x),yaw);
//      Serial.print("yaw"); // left
//      Serial.print(":"); // left
//      Serial.println(convertYaw(event.orientation.x)); //        
        value_l = fabs(control -yaw_val ) / (control - yaw_val);
        value_r = fabs(control +yaw_val ) / (control + yaw_val );
        motor_left((-1)*value_l,fabs(control - yaw_val ) );
        motor_right((-1)*value_r,fabs(control + yaw_val ));
      }
      else
      {
        value_l = fabs(control) /control;
        value_r = fabs(control) /control;
        motor_left((-1)*value_l,fabs(control));
        motor_right((-1)*value_r,fabs(control));
      }
//      Serial.print(value_l); // left
//      Serial.print(","); // left
//      Serial.println(control); //
    }
  }
}
