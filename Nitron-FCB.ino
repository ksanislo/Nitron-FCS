/*
 * Nerf Vortex Nitron - Brushless Fire Control v0.9.0
 */
#include <Servo.h>

// ESC pulse lengths in uS
const int pusherRunPulse = 1590;    // normal run
const int pusherReturnPulse = 1550; // minimum run
const int pusherStopPulse = 1000;   // maximum brake

const int flywheelRunPulse = 1650;  // normal run
const int flywheelStopPulse = 1520; // coast

// Timers for flywheel in mS
const int flywheelSpoolTime = 250;  // 0.25 seconds
const int flywheelRunTime = 250;    // 0.25 seconds

// Time to wait for disc to pop up, in mS
const int pusherDiscDelay = 50;     // 0.05 second

// Button debounce timer, in mS
const int debounceDelay = 5;

// Shot queue depth for each mode
const unsigned int shotCount[] = {1, 3, 65535};

// Arduino pin configuration
const int flywheelRpmPin = 2;
const int pusherRpmPin = 3;
const int magazinePin = 4;
const int ejectPin = 5;
const int disclPin = 6;
const int discrPin = 7;
const int positionPin = 8;
const int triggerPin = 9;
const int readyPin = 10;
const int flywheelEscPin = 11;
const int pusherEscPin = 12;

// ESCs are operated via the Servo library.
Servo flywheel;
Servo pusher;

// global variables
bool magazinePinState = 0;
bool ejectPinState = 0;
bool disclPinState = 0;
bool discrPinState = 0;
bool positionPinState = 0;
bool triggerPinState = 0;
bool readyPinState = 0;
unsigned int shotMode = 1;
unsigned int shotsRemaining = 0;
unsigned long flywheelStartTime = 0;
unsigned long flywheelTimer = 0;
unsigned long positionPinTimer = 0;
int pusherPulse = 1520;
int flywheelPulse = 1520;

void setup() {
  /*
   * This is the setup function which should initialize
   * all additional required features and functions.
   */
   
  pinMode(flywheelRpmPin, INPUT_PULLUP);
  pinMode(pusherRpmPin, INPUT_PULLUP);
  pinMode(magazinePin, INPUT_PULLUP);
  pinMode(ejectPin, INPUT_PULLUP);
  pinMode(disclPin, INPUT_PULLUP);
  pinMode(discrPin, INPUT_PULLUP);
  pinMode(positionPin, INPUT_PULLUP);
  pinMode(triggerPin, INPUT_PULLUP);
  pinMode(readyPin, INPUT_PULLUP);
  flywheel.attach(flywheelEscPin);
  flywheel.writeMicroseconds(flywheelPulse);
  pusher.attach(pusherEscPin);
  pusher.writeMicroseconds(pusherPulse);

  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  /*
   * This is the primary loop.
   */
  
  magazinePinState = true; //debounceInput(magazinePin);
  ejectPinState    = debounceInput(ejectPin);
  disclPinState    = true; //debounceInput(disclPin);
  discrPinState    = true; //debounceInput(discrPin);
  positionPinState = debounceInput(positionPin);
  triggerPinState  = debounceInput(triggerPin);
  readyPinState    = debounceInput(readyPin);
  
  pusherControl();
  flywheelControl();
  fireControl();

  if (positionPinState) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
}

bool debounceInput(int pin) {
  /*
   * To debounce inputs, we'll use some static arrays to 
   * store information on which pin is being checked.
   * 
   * We also need to invert the raw switch values since
   * using INPUT_PULLUP inverts our results.
   */
  
  static unsigned long timer[22] = {0};
  static bool lastState[22] = {0};
  static bool currentState[22] = {0};
  
  bool liveValue = !digitalRead(pin);
  
  if (liveValue != lastState[pin]) {
    timer[pin] = millis();
    lastState[pin] = liveValue;
  }

  if (millis() - timer[pin] >= debounceDelay) {
    currentState[pin] = liveValue;
  }
  
  return currentState[pin];
}

void pusherControl() {
  /* 
   * Operate the pusher motor if we need to retract it,
   * or if we're sending off a shot and the flywheel is spun up.
   * 
   * Failing to chamber a new round in time, disc eject, or magazine removal forces an automatic stop.
   */
  
  //static bool lastPositionPinState = 0;
  
  //if (positionPinState != lastPositionPinState) {
  //  positionPinTimer = millis();
  //  lastPositionPinState = positionPinState;
  //}
  
  if (ejectPinState && magazinePinState) {
    if (positionPinState) {
      if (shotsRemaining) {
        if (disclPinState && discrPinState && flywheelStartTime && millis() - flywheelStartTime >= flywheelSpoolTime) {
          pusherPulse = pusherRunPulse;
        }
      } else {
        pusherPulse = pusherReturnPulse;
      }
    } else {
      if (shotsRemaining) {
        if (flywheelStartTime && millis() - flywheelStartTime >= flywheelSpoolTime) {
          if (shotsRemaining == 1 && shotMode >= 1) { // Back off the pusher speed early for the last shot
            pusherPulse = pusherReturnPulse;
          } else {
            pusherPulse = pusherRunPulse;
          }
        }
        if(!disclPinState && !discrPinState) { // Should be OR, and have a delay timer
          pusherPulse = pusherStopPulse;
        }
      } else {
        pusherPulse = pusherStopPulse;
      }
    }
  } else {
    pusherPulse = pusherStopPulse;
  }
  pusher.writeMicroseconds(pusherPulse);
}

void flywheelControl() {
  /*
   * Start the flywheel on either trigger,
   * Save the initial spool-up time.
   * Maintain flywheel speed for a minimum run time   
   * Disc eject or magazine removal forces an automatic stop.
   */
  
  if ((shotsRemaining || readyPinState) && ejectPinState && magazinePinState){
    if (flywheelStartTime == 0) {
      flywheelStartTime = millis(); // Save the initial spool-up time.
    }
    flywheelTimer = millis(); // Reset the timer.
    flywheelPulse = flywheelRunPulse;
  }
  
  if (millis() - flywheelTimer >= flywheelRunTime || !ejectPinState || !magazinePinState) {
    flywheelPulse = flywheelStopPulse;
    flywheelStartTime = 0;
  }

  flywheel.writeMicroseconds(flywheelPulse);
}

void fireControl() {
  /* 
   * Track trigger and chamber states, count off rounds as they go out.
   * Trigger release, disc eject, or magazine removal forces an automatic zeroing of the queue.
   */
  
  static bool lastTriggerPinState = 0;
  static bool lastpositionPinState = 0;
  
  if (triggerPinState && !lastTriggerPinState && ejectPinState && magazinePinState) {
    shotsRemaining = shotCount[shotMode];
  }
  
  if (shotsRemaining && !lastpositionPinState && positionPinState) {
    shotsRemaining--;
  }
  
  if (!triggerPinState || !ejectPinState || !magazinePinState) {
    shotsRemaining = 0;
  }
  
  lastTriggerPinState = triggerPinState;
  lastpositionPinState = positionPinState;
}
