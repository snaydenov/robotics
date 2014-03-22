// sensors (closer to center -> lower index)
#define left3 A0
#define left2 A1
#define left1 A2
#define right1 A3
#define right2 A4
#define right3 A5

//motors
#define leftForward 5
#define leftBack 6
#define rightForward 10
#define rightBack 11

//boolean
#define True  1
#define False 0

typedef struct SensorsStruct
{
  uint16_t mostLeft;
  uint16_t midLeft;
  uint16_t centLeft;
  
  uint16_t centRight;
  uint16_t midRight;
  uint16_t mostRight;
} Sensors;

typedef enum dir
{
  Forward,
  Back
} Direction;

typedef struct MotorsStruct
{
  uint8_t   leftSpeed;
  Direction leftDir;
  
  uint8_t   leftPrevSpeed;
  Direction leftPrevDir;
  
  uint8_t   rightSpeed;
  Direction rightDir;
  
  uint8_t   rightPrevSpeed;
  Direction rightPrevDir;
} Motors;

Sensors sensors;
Motors motors;
uint16_t threshhold = 500;

void setup()
{
  init(); // +1 greatnessz
}

void loop()
{
  UpdateSensors(&sensors);
  DetermineMotorSpeedAndDirection(&sensors, &motors);
  UpdateMotors(&motors);
}

void UpdateSensors(struct SensorsStruct *sensors)
{
  sensors->mostLeft = analogRead(left3);
  sensors->midLeft = analogRead(left2);
  sensors->centLeft = analogRead(left1);
  sensors->centRight = analogRead(right1);
  sensors->midRight = analogRead(right2);
  sensors->mostRight = analogRead(right3);
}

void DetermineMotorSpeedAndDirection(
          struct SensorsStruct *sensors, 
          struct MotorsStruct *motors
          )
{
  if(sensors->midLeft > threshhold)
  {
    motors->leftSpeed = 150;
    motors->leftDir = Forward;
    motors->rightSpeed = 255;
    motors->rightDir = Forward;
  }
  
  if(sensors->midRight > threshhold)
  {
    motors->leftSpeed = 255;
    motors->leftDir = Forward;
    motors->rightSpeed = 150;
    motors->rightDir = Forward;
  }
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

void init()
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
}
