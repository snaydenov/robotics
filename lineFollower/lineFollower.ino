#line 1 "lineFollower.ino"
// sensors (closer to center -> lower index)
#define left3 A0
#define left2 A1
#define left1 A2
#define right1 A3
#define right2 A4
#define right3 A5

//motors
#define leftForward 6
#define leftBack 5
#define rightForward 9
#define rightBack 10

//boolean
#define True  1
#define False 0

//pid coeficients
#define p_coef 20 
#define d_coef 0

//constraints
#define maximum 125
#define maxSensorValue 50

#include "Arduino.h"
void setup();
void loop();
void UpdateSensors(struct SensorsStruct *sensors);
void DetermineMotorSpeedAndDirection(
          struct SensorsStruct *sensors, 
          struct MotorsStruct *motors
          );
void CalibrateSensors(struct SensorsStruct* sensors);
int ReadLine(struct SensorsStruct* sensors);
void UpdateMotors(struct MotorsStruct *motors);
bool MotorsNeedUpdate(struct MotorsStruct *motors);
void initialize();
#line 27
struct SensorsStruct
{
  uint16_t left_0;
  uint16_t left_1;
  uint16_t left_2;
  uint16_t right_2;
  uint16_t right_1;
  uint16_t right_0;
};

enum Direction
{
  Forward,
  Back
};

struct MotorsStruct
{
  uint8_t   leftSpeed;
  Direction leftDir;
  
  uint8_t   leftPrevSpeed;
  Direction leftPrevDir;
  
  uint8_t   rightSpeed;
  Direction rightDir;
  
  uint8_t   rightPrevSpeed;
  Direction rightPrevDir;
};

struct SensorsStruct sensors;
struct MotorsStruct motors;
uint16_t threshhold = 500;
int last_proportional = 0;
void setup()
{
  Serial.begin(9600);
  initialize(); // +1 greatness
  
}

void loop()
{
  
  UpdateSensors(&sensors);
  DetermineMotorSpeedAndDirection(&sensors, &motors);
  UpdateMotors(&motors);
  
  
}

void UpdateSensors(struct SensorsStruct *sensors)
{
  sensors->left_0 = analogRead(left3);
  sensors->left_1 = analogRead(left2);
  sensors->left_2 = analogRead(left1);
  sensors->right_2 = analogRead(right1);
  sensors->right_1 = analogRead(right2);
  sensors->right_0 = analogRead(right3);
}

void DetermineMotorSpeedAndDirection(
          struct SensorsStruct *sensors, 
          struct MotorsStruct *motors
          )
{
  int proportional = ReadLine(sensors);
  
  
  
  // Compute the derivative (change) and integral (sum) of the
  // position.
  int derivative = proportional - last_proportional;

  last_proportional = proportional;
  

  int power_difference = proportional/p_coef + derivative*d_coef;

  if (power_difference > maximum)
    power_difference = maximum;
  if (power_difference < -maximum)
    power_difference = -maximum;
  
  if (power_difference < 0)
  {
    motors -> rightSpeed = maximum + power_difference;
    motors -> leftSpeed = maximum - power_difference/3;
  }
  else
  {
    motors -> rightSpeed = maximum +power_difference/3;
    motors -> leftSpeed = maximum - power_difference;
  }
}

void CalibrateSensors(struct SensorsStruct* sensors)
{
  sensors -> left_2 = sensors -> left_2*2;
  sensors -> left_1 = sensors -> left_1*2;
  sensors -> left_0 = sensors -> left_0*2;
  sensors -> right_2 = sensors -> right_2*2;
  sensors -> right_1 = sensors -> right_1*2;
  sensors -> right_0 = sensors -> right_0*2;
}

int ReadLine(struct SensorsStruct* sensors)
{
	unsigned char i, on_line = 0;
	unsigned long avg; // this is for the weighted total, which is long
	                   // before division
	float sum; // this is for the denominator which is <= 64000
	static int last_value=0; // assume initially that the line is left.

	CalibrateSensors(sensors);
        
	avg = 0;
	sum = 0;
        int sensor_values[6];
        sensor_values[0] = sensors -> left_0;
        sensor_values[1] = sensors -> left_1;
        sensor_values[2] = sensors -> left_2;
        sensor_values[3] = sensors -> right_2;
        sensor_values[4] = sensors -> right_1;
        sensor_values[5] = sensors -> right_0;
	for(i=0;i<6;i++) {
		int value = sensor_values[i];
		
		// keep track of whether we see the line at all
		if(value > 200) {
                        
                        
			on_line = 1;
		}
		
		// only average in values that are above a noise threshold
		if(value > 20) {
			avg += (long)(value) * (i);
			sum += (float)value/1000;
		}
	}
        
	if(!on_line)
	{
		// If it last read to the left of center, return 0.
		if(last_value < (5)*1000/2)
			return -2500;
		
		// If it last read to the right of center, return the max.
		else
			return 2500;

	}
        
	last_value = avg/sum;
        return last_value - 2500;
}

void UpdateMotors(struct MotorsStruct *motors)
{
  if(False != MotorsNeedUpdate(motors))
  {
    if(Forward == motors->leftDir)
    {
      analogWrite(leftForward, motors->leftSpeed);
      digitalWrite(leftBack, LOW);
    }
    else if(Back == motors->leftDir)
    {
      analogWrite(leftBack, motors->leftSpeed);
      digitalWrite(leftForward, LOW);
    }
    
    if(Forward == motors->rightDir)
    {
      analogWrite(rightForward, motors->rightSpeed);
      digitalWrite(rightBack, LOW);
    }
    else if(Back == motors->rightDir)
    {
      analogWrite(rightBack, motors->rightSpeed);
      digitalWrite(rightForward, LOW);
    }
    
    motors->leftPrevDir = motors->leftDir;
    motors->leftPrevSpeed = motors->leftSpeed;
    motors->rightPrevDir = motors->rightDir;
    motors->rightPrevSpeed = motors->rightSpeed;
  }
}

bool MotorsNeedUpdate(struct MotorsStruct *motors)
{
  if(motors->leftSpeed != motors->leftPrevSpeed)
  {
    return True;
  }
  if(motors->rightSpeed != motors->rightPrevSpeed)
  {
    return True;
  }
  if(motors->leftDir != motors->leftPrevDir)
  {
    return True;
  }
  if(motors->rightDir != motors->rightPrevDir)
  {
    return True;
  }
  
  return False;
}

void initialize()
{
  // port initialisation
  DDRD &= 3;// 0 and 1 are serial pins 3 = 0000 0011 in binary
  PORTD &= 3;
  DDRB = 0;
  PORTB = 0;
  DDRC = 0;
  PORTC = 0;
  
  // sensors
  pinMode(left3, INPUT);
  pinMode(left2, INPUT);
  pinMode(left1, INPUT);
  pinMode(right1, INPUT);
  pinMode(right2, INPUT);
  pinMode(right3, INPUT);
  
  // motors
  pinMode(leftForward, OUTPUT);
  pinMode(leftBack, OUTPUT);
  pinMode(rightForward, OUTPUT);
  pinMode(rightBack, OUTPUT);
  
  memset((void *) &sensors, 0, sizeof(sensors));
  memset((void *) &motors, 0, sizeof(motors));
  
  motors.leftDir = Forward;
  motors.rightDir = Forward;
}


