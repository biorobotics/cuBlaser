#define SRC_NEUTRAL 1514
#define SRC_MAX 2044
#define SRC_MIN 980
#define TRC_NEUTRAL 1514
#define TRC_MAX 2044
#define TRC_MIN 980
#define RC_DEADBAND 30
#define RC_DEADBAND_RIGHT 10
#define ERROR_center 30
#define pERROR 0  

uint16_t unSteeringMin = SRC_MIN + pERROR;
uint16_t unSteeringMax = SRC_MAX - pERROR;
uint16_t unSteeringCenter = SRC_NEUTRAL;

uint16_t unThrottleMin = TRC_MIN + pERROR;
uint16_t unThrottleMax = TRC_MAX - pERROR;
uint16_t unThrottleCenter = TRC_NEUTRAL;

#define PWM_MIN 0
#define PWM_MAX 255

#define GEAR_NONE 1
#define GEAR_IDLE 1
#define GEAR_FULL 2

#define PWM_SPEED_LEFT 19
#define PWM_SPEED_RIGHT 14
#define LEFT1 18
#define LEFT2 17
#define RIGHT1 16
#define RIGHT2 15

// Assign your channel in pins
#define THROTTLE_IN_PIN 22
#define THROTTLE_INT 1

#define STEERING_IN_PIN 23
#define STEERING_INT 0

// These bit flags are set in bUpdateFlagsShared to indicate which
// channels have new signals
#define THROTTLE_FLAG 1
#define STEERING_FLAG 2

// holds the update flags defined above
volatile uint8_t bUpdateFlagsShared;

// shared variables are updated by the ISR and read by loop.
volatile uint16_t unThrottleInShared;
volatile uint16_t unSteeringInShared;

// These are used to record the rising edge of a pulse in the calcInput functions
uint32_t ulThrottleStart;
uint32_t ulSteeringStart;

uint8_t gThrottle = 0;
uint8_t gGear = GEAR_NONE;
uint8_t gOldGear = GEAR_NONE;

#define DIRECTION_STOP 0
#define DIRECTION_FORWARD 1
#define DIRECTION_REVERSE 2
#define DIRECTION_ROTATE_RIGHT 3
#define DIRECTION_ROTATE_LEFT 4

uint8_t gThrottleDirection = DIRECTION_STOP;
uint8_t gDirection = DIRECTION_STOP;
uint8_t gOldDirection = DIRECTION_STOP;

#define IDLE_MAX 5

#define MODE_RUN 0


uint8_t gMode = MODE_RUN;

unsigned long pulse_time;




void setup() {
  Serial.begin(9600);

  Serial.println("start");
  analogWriteResolution(8); // 0 - 255
  pinMode(STEERING_IN_PIN, INPUT);
  pinMode(THROTTLE_IN_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(THROTTLE_IN_PIN), calcThrottle, CHANGE);
  attachInterrupt(digitalPinToInterrupt(STEERING_IN_PIN), calcSteering, CHANGE);

  pinMode(PWM_SPEED_LEFT,OUTPUT);  // ENA
  pinMode(PWM_SPEED_RIGHT,OUTPUT); // ENB
  // Set all the motor control pins to outputs
  pinMode(LEFT1,OUTPUT);           // in1
  pinMode(LEFT2,OUTPUT);           // in2
  pinMode(RIGHT1,OUTPUT);          // in3
  pinMode(RIGHT2,OUTPUT);          // in4
  
  // Turn off motors - Initial state
  digitalWrite(LEFT1, LOW);
  digitalWrite(LEFT2, LOW);
  digitalWrite(RIGHT1, LOW);
  digitalWrite(RIGHT2, LOW);

  pulse_time = millis();
}

void loop() {
  /*
  analogWrite(PWM_SPEED_LEFT, 250);
  analogWrite(PWM_SPEED_RIGHT, 130);
  digitalWrite(RIGHT1,HIGH);
      digitalWrite(RIGHT2,LOW);
      digitalWrite(LEFT1,HIGH);
      digitalWrite(LEFT2,LOW);
      //delay (20);
  
  */
  static uint16_t unThrottleIn;
  static uint16_t unSteeringIn;
  // local copy of update flags
  static uint8_t bUpdateFlags;
  // fail_safe();
  // check shared update flags to see if any channels have a new signal
  if(bUpdateFlagsShared){
    noInterrupts(); 
    pulse_time =millis() ;
    // take a local copy 
    bUpdateFlags = bUpdateFlagsShared;

    if(bUpdateFlags & THROTTLE_FLAG){
      unThrottleIn = unThrottleInShared;
    }

    if(bUpdateFlags & STEERING_FLAG){
      unSteeringIn = unSteeringInShared;
    }

    bUpdateFlagsShared = 0;

    interrupts(); 
  }


  // processing
  // only use the local values unAuxIn, unThrottleIn and unSteeringIn
  if(gMode == MODE_RUN){
    if(bUpdateFlags & THROTTLE_FLAG){
      unThrottleIn = constrain(unThrottleIn,unThrottleMin,unThrottleMax);
      
      if(unThrottleIn > (unThrottleCenter + ERROR_center)){
        gThrottle = map(unThrottleIn,(unThrottleCenter + ERROR_center),unThrottleMax,PWM_MIN,PWM_MAX);
        gThrottleDirection = DIRECTION_FORWARD;
      }
      else if (unThrottleIn < (unThrottleCenter - ERROR_center)){
        gThrottle = map(unThrottleIn,unThrottleMin,(unThrottleCenter- ERROR_center),PWM_MAX,PWM_MIN);
        gThrottleDirection = DIRECTION_REVERSE;
      }
 
      else{
        gThrottleDirection = DIRECTION_STOP;
        gThrottle = 0;
      }
  
      if(gThrottle < IDLE_MAX){
        gGear = GEAR_IDLE;
      }
      else{
        gGear = GEAR_FULL;
      }
    }

    uint8_t throttleLeft = gThrottle;
    uint8_t throttleRight = gThrottle;
    if(bUpdateFlags & STEERING_FLAG){
  
      gDirection = gThrottleDirection;
      
      // see previous comments regarding trapping out of range errors
      // this is left for the user to decide how to handle and flag
      unSteeringIn = constrain(unSteeringIn,unSteeringMin,unSteeringMax);
  
      // if idle spin on spot
      switch(gGear){
      case GEAR_IDLE:
        if(unSteeringIn > (unSteeringCenter + RC_DEADBAND)){
          gDirection = DIRECTION_ROTATE_RIGHT;
          // use steering to set throttle
          throttleRight = throttleLeft = map(unSteeringIn,unSteeringCenter,unSteeringMax,PWM_MIN,PWM_MAX);
        }
        else if(unSteeringIn < (unSteeringCenter - RC_DEADBAND)){
          gDirection = DIRECTION_ROTATE_LEFT;
          // use steering to set throttle
          throttleRight = throttleLeft = map(unSteeringIn,unSteeringMin,unSteeringCenter,PWM_MAX,PWM_MIN);
        }
        break;
      // if not idle proportionally restrain inside track to turn vehicle around it
      case GEAR_FULL:
        if(unSteeringIn > (unSteeringCenter + RC_DEADBAND)){
          throttleLeft = map(unSteeringIn,unSteeringCenter,unSteeringMax,gThrottle,PWM_MIN);
        }
        else if(unSteeringIn < (unSteeringCenter - RC_DEADBAND)){
          throttleRight = map(unSteeringIn,unSteeringMin,unSteeringCenter,PWM_MIN,gThrottle);
        }
        break;
      }
      analogWrite(PWM_SPEED_LEFT,throttleLeft);
      analogWrite(PWM_SPEED_RIGHT,throttleRight);
    }
  }
  
  if((gDirection != gOldDirection) || (gGear != gOldGear)){
    gOldDirection = gDirection;
    gOldGear = gGear;

    digitalWrite(RIGHT1,LOW);
    digitalWrite(RIGHT2,LOW);
    digitalWrite(LEFT1,LOW);
    digitalWrite(LEFT2,LOW);

    switch(gDirection){
    case DIRECTION_REVERSE:
      digitalWrite(RIGHT1,LOW);
      digitalWrite(RIGHT2,HIGH);
      digitalWrite(LEFT1,LOW);
      digitalWrite(LEFT2,HIGH);
      break;
    case DIRECTION_FORWARD:
      digitalWrite(RIGHT1,HIGH);
      digitalWrite(RIGHT2,LOW);
      digitalWrite(LEFT1,HIGH);
      digitalWrite(LEFT2,LOW);
      
      break;
    case DIRECTION_ROTATE_RIGHT:
      digitalWrite(LEFT1,HIGH);
      digitalWrite(LEFT2,LOW);
      digitalWrite(RIGHT1,LOW);
      digitalWrite(RIGHT2,HIGH);
      break;
    case DIRECTION_ROTATE_LEFT:
      digitalWrite(LEFT1,LOW);
      digitalWrite(LEFT2,HIGH);
      digitalWrite(RIGHT1,HIGH);
      digitalWrite(RIGHT2,LOW);
      break;
    case DIRECTION_STOP:
      digitalWrite(LEFT1,LOW);
      digitalWrite(LEFT2,LOW);
      digitalWrite(RIGHT1,LOW);
      digitalWrite(RIGHT2,LOW);
      break;
    }
  }

  bUpdateFlags = 0;
  
}

void calcThrottle()
{
  // if the pin is high, its a rising edge of the signal pulse, so lets record its value
  if(digitalRead(THROTTLE_IN_PIN) == HIGH)
  {
    ulThrottleStart = micros();
  }
  else
  {
    // else it must be a falling edge, so lets get the time and subtract the time of the rising edge
    // this gives use the time between the rising and falling edges i.e. the pulse duration.
    unThrottleInShared = (uint16_t)(micros() - ulThrottleStart);
    // use set the throttle flag to indicate that a new throttle signal has been received
    bUpdateFlagsShared |= THROTTLE_FLAG;
  }
}

void calcSteering()
{
  if(digitalRead(STEERING_IN_PIN) == HIGH)
  {
    ulSteeringStart = micros();
  }
  else
  {
    unSteeringInShared = (uint16_t)(micros() - ulSteeringStart);
    bUpdateFlagsShared |= STEERING_FLAG;
  }
}
