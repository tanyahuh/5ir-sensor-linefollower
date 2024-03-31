#define left_motor_forward 13
#define left_motor_backward 12
#define left_motor_pwm 27//enabler pins
#define right_motor_pwm 33//enabler pins 
#define right_motor_forward 25
#define right_motor_backward 23
#define sensor1 2
#define sensor2 4
#define sensor3 5
#define sensor4 18
#define sensor5 19
#define Kp 0 //set Kp Value
#define Ki 0 //set Ki Value
#define Kd 0 //set Kd Value

int proportional=0;
int integral=0;
int derivative=0;
int last_proportional=0;
int right_speed=0;
int left_speed=0;
int sensor_sum=0;
int sensor[5]={0,0,0,0,0};  
int sensorPins[5] = {2,4,5,18,19}; 
int sensor_average=0;
int position=0;
int error_value=0;
int base_speed=255;
int correction;
int set_point;

void pid_calc();
void calc_turn();
void motor_drive(int , int );

void setup()
{
  //sensors
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(sensor4, INPUT);
  pinMode(sensor5, INPUT);

  //motors
  pinMode(right_motor_forward, OUTPUT);
  pinMode(right_motor_backward, OUTPUT);
  pinMode(left_motor_forward, OUTPUT);
  pinMode(left_motor_backward, OUTPUT);

  Serial.begin(9600);
  
  //finding set point

  while(true)
  {
    for (int i = -2; i <= 2; i++)// reference code had limits -2 to 2 i did not understand why that was necessary
    {
      sensor[i+2] = analogRead(sensorPins[i+2]);
      sensor_average += sensor[i+2] * i * 1000;   //weighted mean   
      sensor_sum += int(sensor[i+2]);
    }
    position = int(sensor_average / sensor_sum);
  
    Serial.print(sensor_average);
    Serial.print(' ');
    Serial.print(sensor_sum);
    Serial.print(' ');
    Serial.print(position);
    Serial.println();
    delay(2000);
  }

  set_point = position;
}


void loop()
{
  pid_calc();
  calc_turn();
}

void pid_calc()
{
  for(int i=-2; i<=2; i++){
    sensor[i+2]=analogRead(sensorPins[i+2]);
    sensor_average = sensor[i+2]*i*1000; //weighted mean
    sensor_sum += sensor[i+2];
  }

  position= int(sensor_average/sensor_sum);

  error_value= set_point -position;

  proportional = error_value;
  integral += proportional;
  derivative = proportional - last_proportional;
  
  last_proportional=proportional;

  correction = int(Kp*proportional + Ki*integral + Kd*derivative);

 }

void calc_turn()
{

if (error_value< -256)     
  { 
error_value = -256; 
  }  

if (error_value> 256) 
  { 
error_value = 256; 
  }  

if (error_value< 0) 
  { 
left_speed= base_speed + error_value ; 
right_speed = base_speed ; 
  } 

else 
  { 
left_speed = base_speed; 
right_speed = base_speed - error_value; 
  }

  //restricting speeds of motors between 255 and -255
  
if (right_speed > 255){
    right_speed = 255;}
    
if (left_speed > 255)
{
    left_speed = 255;}
    
if (right_speed < -255)
{
    right_speed = -255;}
    
if (left_speed < -255) 
{
    left_speed = -255;}
 
 motor_drive(right_speed,left_speed);  
}

void motor_drive(int right, int left)
{
  
  if(right>0)
  {
    analogWrite(right_motor_forward, right);   
    analogWrite(right_motor_backward, 0);
  }
  else
  {
    analogWrite(right_motor_forward, 0); 
    analogWrite(right_motor_backward, abs(right));
  }
  
 
  if(left>0)
  {
    analogWrite(left_motor_forward, left);
    analogWrite(left_motor_backward, 0);
  }
  else 
  {
    analogWrite(left_motor_forward, 0);
    analogWrite(left_motor_backward, abs(left));
  }
  }


