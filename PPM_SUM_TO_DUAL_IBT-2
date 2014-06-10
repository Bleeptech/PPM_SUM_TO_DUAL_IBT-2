
static const int NumberOfChannels = 8;          // Change to match number of channels
//static const int InputPin = 21;                // Pin associated with interrupt 2 on Arduino Mega2560
static const int InputPin = 21;                // Pin associated with interrupt 2 on Arduino Mega2560
static const int FrameSpace = 8000;   // 8 ms
static const int PPM_MIN = 900;
static const int PPM_MAX = 2100;

unsigned long Current_Time = 0;
unsigned long Last_Spike = 0;

#define PROG_DELAY 10
#define RC_MIN 900
#define RC_MAX 2100
#define RC_NEUTRAL 1500
#define RC_DEADBAND 50
#define IDLE_MAX 25
#define STICK_ERROR 20

#define PWM_MIN 0
#define PWM_MAX 200
#define PWM_LIMIT 100
#define PWM_DIV 2

#define GEAR_NONE 0
#define GEAR_IDLE 1
#define GEAR_FULL 2

const int A_LPWM = 9; // H-bridge leg 1 ->LPWM
const int A_L_EN = 8; // H-bridge enable pin 1 -> L_EN
const int A_RPWM = 6; // H-bridge leg 2 ->RPWM
const int A_R_EN = 7; // H-bridge enable pin 2 -> R_EN

const int B_LPWM =11; // H-bridge leg 1 ->LPWM
const int B_L_EN = 4; // H-bridge enable pin 1 -> L_EN
const int B_RPWM = 10; // H-bridge leg 2 ->RPWM
const int B_R_EN = 12; // H-bridge enable pin 2 -> R_EN

const int PIN_BRAKES = 5; // Control relay for brakes on wheelchair motors
const int PIN_HORN = A3; // Control relay 1 for winch motor
const int WINCH_IN = A4; // Control relay 1 for winch motor
const int WINCH_OUT = A5; // Control relay 2 for winch motor

int PWM_SPEED_PORT = PWM_LIMIT; 
int PWM_SPEED_STARBOARD= PWM_LIMIT;

int throttleLeft = 0;
int throttleRight = 0;

#define FRAME 0
#define HORN 1
#define THROTTLE 2
#define STEERING 3
#define WINCH 4
#define AUX0 5
#define AUX1 6
#define AUX2 7
#define AUX3 8

int x = 0;
int Count = 0;
int Current_Channel = 0;
int Spike_Length[NumberOfChannels + 1];
int Last_Spike_Length[NumberOfChannels + 1];
int LastChannelFlag = false;

int i = 0;
int TC = 0;
int SC = 0;
int ON = 0;
int OFF = 1;

uint16_t unSteeringMin = RC_MIN;
uint16_t unSteeringMax = RC_MAX;
uint16_t unSteeringCenter = RC_NEUTRAL;

uint16_t unThrottleMin = RC_MIN;
uint16_t unThrottleMax = RC_MAX;
uint16_t unThrottleCenter = RC_NEUTRAL;

uint16_t unWinchMin = RC_MIN;
uint16_t unWinchMax = RC_MAX;
uint16_t unWinchCenter = RC_NEUTRAL;

uint16_t unHornMin = RC_MIN;
uint16_t unHornMax = RC_MAX;
uint16_t unHornCenter = RC_NEUTRAL;

uint8_t gThrottle = 0;
uint8_t gGear = GEAR_NONE;
uint8_t gOldGear = GEAR_NONE;

#define DIRECTION_STOP 0
#define DIRECTION_FORWARD 1
#define DIRECTION_REVERSE 2
#define DIRECTION_ROTATE_STARBOARD 3
#define DIRECTION_ROTATE_PORT 4

uint8_t gThrottleDirection = DIRECTION_STOP;
uint8_t gDirection = DIRECTION_STOP;
uint8_t gOldDirection = DIRECTION_STOP;

#define MODE_RUN 0

// uint8_t gMode = MODE_RUN;

// create local variables to hold a local copies of the channel inputs these are declared static so that their 
// values will be retained between calls to loop.
static uint16_t unThrottleIn = RC_NEUTRAL;
static uint16_t unSteeringIn = RC_NEUTRAL;
static uint16_t unWinchIn = RC_NEUTRAL;
static uint16_t unHornIn = RC_NEUTRAL;
// local copy of update flags
//  static uint8_t bUpdateFlags;
// ############################################################################################

void setup()
{
  Serial.print(" SETUP ");
  delay(100);
//  attachInterrupt( 2, Spike, RISING);
  attachInterrupt(2, Spike, RISING);
  Last_Spike = micros();
  Serial.begin(115200);
  pinMode(A_LPWM, OUTPUT);
  pinMode(A_RPWM, OUTPUT);
  pinMode(A_L_EN, OUTPUT);
  pinMode(A_R_EN, OUTPUT);
  pinMode(B_LPWM, OUTPUT);
  pinMode(B_RPWM, OUTPUT);
  pinMode(B_L_EN, OUTPUT);
  pinMode(B_R_EN, OUTPUT);
  pinMode(PIN_BRAKES, OUTPUT);
  pinMode(PIN_HORN, OUTPUT);
  pinMode(WINCH_IN, OUTPUT);
  pinMode(WINCH_OUT, OUTPUT);

  digitalWrite(A_L_EN, HIGH);
  digitalWrite(A_R_EN, HIGH);
  digitalWrite(B_L_EN, HIGH);
  digitalWrite(B_R_EN, HIGH);

  digitalWrite(PIN_BRAKES, OFF);
  digitalWrite(WINCH_IN, OFF);
  digitalWrite(WINCH_OUT, OFF);
  digitalWrite(PIN_HORN, OFF);
  preTest();
}

void preTest()
{
  digitalWrite(PIN_BRAKES, ON);
  delay(50); 
  digitalWrite(PIN_BRAKES, OFF);
  delay(50); 
  digitalWrite(WINCH_IN, ON);
  delay(50); 
  digitalWrite(WINCH_IN, OFF);
  delay(50); 
  digitalWrite(WINCH_OUT, ON);
  delay(50); 
  digitalWrite(WINCH_OUT, OFF);
  delay(50); 
  digitalWrite(PIN_HORN, ON);
  delay(50); 
  digitalWrite(PIN_HORN, OFF);
  gDirection = DIRECTION_STOP;
}

// ########################################################################################

void loop()
{
    Count++;

    if (Count >= 50)
    {
      Count = 0;
      SHUTDOWN();
    }
    else 
    {
      if (LastChannelFlag == true)  // If we are on the last Spike
      {
            LastChannel();               // Get the last pulse value
            Display();
            unThrottleIn = Spike_Length[THROTTLE];
            unSteeringIn = Spike_Length[STEERING];
            unWinchIn = Spike_Length[WINCH];
            unHornIn = Spike_Length[HORN];
            calcDrive();
            runBrakes();
            runDrive();
            runHorn();
            runWinch();
//            delay(100);
    }
  }
}  //end loop()

// ############################################################################################
// Stores the length of the spike in Spike_Length. 
// Framespace length in stored in SpikeLength[0]

void Spike()
{
//  Count ++;
  Current_Time = micros();
  Spike_Length[Current_Channel] = Current_Time - Last_Spike;

  if (Spike_Length[Current_Channel] > FrameSpace)
  {
    Current_Channel = 0;       // oops, you were actually in the frame space --- need to add correction
    Count = 0; //Reset Failsafe
  } 
  if ((Spike_Length[Current_Channel] < PPM_MIN) || (Spike_Length[Current_Channel] > PPM_MAX))
  {
    Spike_Length[Current_Channel] = Last_Spike_Length[Current_Channel];
    Count = Count + 1;
  }     
  else
  {
    Last_Spike_Length[Current_Channel] = Spike_Length[Current_Channel];
    Count = Count - 1;
  }
  Last_Spike = Current_Time;                             // Set the current time as the previous time to the next one.
  Current_Channel = Current_Channel + 1;                 // Reading the next channel now
  if (Current_Channel == NumberOfChannels) 
  {
    LastChannelFlag = true;
  }

}  

// ############################################################################################

void LastChannel()
{ 
  while(digitalRead(InputPin) == LOW);  
  { //;  
  Current_Time = micros();  // Now it has fallen
  Spike_Length[Current_Channel] = Current_Time - Last_Spike;    
  Current_Channel = 0;
  Last_Spike = Current_Time;  
  LastChannelFlag = false; 
//  Count = 0;
  }
}

// ############################################################################################

void Display()
{
  for ( x = 1; x <= NumberOfChannels; x++)
  {
    Serial.print("CH-");
    Serial.print(x);
    Serial.print(" ");
     if ((Spike_Length[Current_Channel] < PPM_MIN) || (Spike_Length[Current_Channel] > PPM_MAX))
     {
       Serial.print(" ERROR ");
     }
     else
     {
       Serial.print(Spike_Length[x]);      // Mapping values may need to be changed depending on receiver
       Serial.print(" : ");
      }
    }
    Serial.println();
    delay(100);
}

// ############################################################################################

void calcDrive()
{
  unThrottleIn = constrain(unThrottleIn,unThrottleMin,unThrottleMax);
  if(unThrottleIn > unThrottleCenter + (IDLE_MAX/2) + STICK_ERROR )
  {
    gThrottle = map(unThrottleIn,unThrottleCenter,unThrottleMax,PWM_MIN,PWM_MAX);
    gThrottleDirection = DIRECTION_FORWARD;
  }
  else if(unThrottleIn < unThrottleCenter - (IDLE_MAX/2) - STICK_ERROR )

  {
    gThrottle = map(unThrottleIn,unThrottleMin,unThrottleCenter,PWM_MAX,PWM_MIN);
    gThrottleDirection = DIRECTION_REVERSE;
  }

  if(gThrottle < (IDLE_MAX + STICK_ERROR))
  {
    gGear = GEAR_IDLE;
  }
  else
  {
    gGear = GEAR_FULL;      
  }
  throttleLeft = gThrottle;
  throttleRight = gThrottle;
  gDirection = gThrottleDirection;

  unSteeringIn = constrain(unSteeringIn,unSteeringMin,unSteeringMax);

  // if idle spin on spot
  switch(gGear)
  {
  case GEAR_IDLE:
    if(unSteeringIn > (unSteeringCenter + RC_DEADBAND) + STICK_ERROR )
    {
      gDirection = DIRECTION_ROTATE_STARBOARD;
      // use steering to set throttle
      throttleRight = throttleLeft = map(unSteeringIn,unSteeringCenter,unSteeringMax,PWM_MIN,PWM_MAX);
    }
    else if(unSteeringIn < (unSteeringCenter - RC_DEADBAND) - STICK_ERROR)
    {
      gDirection = DIRECTION_ROTATE_PORT;
      // use steering to set throttle
      throttleRight = throttleLeft = map(unSteeringIn,unSteeringMin,unSteeringCenter,PWM_MAX,PWM_MIN);
    }
    else if((unSteeringIn < (unSteeringCenter + RC_DEADBAND/2)) && (unSteeringIn > (unSteeringCenter - RC_DEADBAND/2)))
    {
      gDirection = DIRECTION_STOP;
    }
    break;

    // if not idle proportionally restrain inside track to turn vehicle around it
  case GEAR_FULL:
    if(unSteeringIn > (unSteeringCenter + RC_DEADBAND))
    {
      throttleRight = map(unSteeringIn,unSteeringCenter,unSteeringMax,gThrottle,PWM_MIN);
    }
    else if(unSteeringIn < (unSteeringCenter - RC_DEADBAND))
    {
      throttleLeft = map(unSteeringIn,unSteeringMin,unSteeringCenter,PWM_MIN,gThrottle);
    }
    break;
  }
  PWM_SPEED_PORT = throttleLeft;
  PWM_SPEED_STARBOARD = throttleRight;
}


// ########################################################################################
// #############                            BRAKES                            #############
// ########################################################################################
void runBrakes()
{
  if (unThrottleIn < (unThrottleCenter + STICK_ERROR) && (unThrottleIn > (unThrottleCenter - STICK_ERROR)))
  { 
    TC = 1; 
  }
  else
  { 
    TC = 0; 
  }

  if (unSteeringIn < (unSteeringCenter + STICK_ERROR) && (unSteeringIn > (unSteeringCenter - STICK_ERROR)))
  { 
    SC = 1; 
  }
  else
  { 
    SC = 0; 
  }


  if((TC == 1 ) && (SC == 1))
  { 
    digitalWrite(PIN_BRAKES, OFF); 
    gDirection = DIRECTION_STOP; 
  }
  else
  { 
    digitalWrite(PIN_BRAKES, ON); 
  }
}
// ########################################################################################

void runDrive()
{
  //  if((gDirection != gOldDirection) || (gGear != gOldGear))
  //  if((gDirection != gOldDirection))
  {
    gOldDirection = gDirection;
    gOldGear = gGear;
    switch(gDirection)
    {
    case DIRECTION_FORWARD:
      analogWrite(A_RPWM,PWM_SPEED_PORT); //pwm value
      digitalWrite(A_LPWM, LOW);
      analogWrite(B_RPWM,PWM_SPEED_STARBOARD); //pwm value
      digitalWrite(B_LPWM, LOW);
      break;
    case DIRECTION_REVERSE:
      digitalWrite(A_RPWM, LOW);
      analogWrite(A_LPWM, PWM_SPEED_PORT); //pwm value
      digitalWrite(B_RPWM, LOW);
      analogWrite(B_LPWM, PWM_SPEED_STARBOARD); //pwm value
      break;
    case DIRECTION_ROTATE_PORT:
      PWM_SPEED_PORT = PWM_SPEED_PORT/PWM_DIV;
      PWM_SPEED_STARBOARD = PWM_SPEED_STARBOARD/PWM_DIV;
      digitalWrite(A_RPWM, LOW);
      analogWrite(A_LPWM, PWM_SPEED_PORT); //pwm value
      analogWrite(B_RPWM,PWM_SPEED_STARBOARD); //pwm value
      digitalWrite(B_LPWM, LOW);
      break;
    case DIRECTION_ROTATE_STARBOARD:
      PWM_SPEED_PORT = PWM_SPEED_PORT/PWM_DIV;
      PWM_SPEED_STARBOARD = PWM_SPEED_STARBOARD/PWM_DIV;
      analogWrite(A_RPWM,PWM_SPEED_PORT); //pwm value
      digitalWrite(A_LPWM, LOW);
      digitalWrite(B_RPWM, LOW);
      analogWrite(B_LPWM, PWM_SPEED_STARBOARD); //pwm value
      break;
    case DIRECTION_STOP:
      digitalWrite(A_RPWM, LOW);
      digitalWrite(A_LPWM, LOW);
      digitalWrite(B_RPWM, LOW);
      digitalWrite(B_LPWM, LOW);
      break;
    }
  }
}
// ########################################################################################

void runWinch()
{
  if (Spike_Length[WINCH] < 1200) { 
    digitalWrite(WINCH_IN, ON); 
  }
  else { 
    digitalWrite(WINCH_IN, OFF); 
  }

  if (Spike_Length[WINCH] > 1800) { 
    digitalWrite(WINCH_OUT, ON); 
  }
  else { 
    digitalWrite(WINCH_OUT, OFF); 
  } 
}

// ########################################################################################

void runHorn()
{
  if (Spike_Length[HORN] > 1500) { 
    digitalWrite(PIN_HORN, ON); 
  }
  else { 
    digitalWrite(PIN_HORN, OFF); 
  }
}

// ############################################################################################

void SHUTDOWN()
{
  Serial.println("####  SIGNAL LOSS  ####");
  gDirection = DIRECTION_STOP;
  digitalWrite(A_RPWM, LOW);
  digitalWrite(A_LPWM, LOW);
  digitalWrite(B_RPWM, LOW);
  digitalWrite(B_LPWM, LOW);
  digitalWrite(PIN_BRAKES, OFF);
  delay(500);
}


