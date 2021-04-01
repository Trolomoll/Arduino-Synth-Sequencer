#include <Arduino.h>

// Auduino Sequencer, the Lo-Fi granular 8-step sequencer
// Modified by NPoole @ SparkFun Electronics ( http://sparkfun.com )
// Based on the Auduino Synthesizer (v5) by Peter Knight, tinker.it ( http://tinker.it )
// 
// Auduino Info: http://code.google.com/p/tinkerit/wiki/Auduino
// Sequencer Hardware Tutorial: https://learn.sparkfun.com/tutorials/step-sequencing-with-auduino
// 
// Enjoy!

#include <avr/io.h>
#include <avr/interrupt.h>
#include <LiquidCrystal.h>

uint16_t syncPhaseAcc;
uint16_t syncPhaseInc;
uint16_t grainPhaseAcc;
uint16_t grainPhaseInc;
uint16_t grainAmp;
uint8_t grainDecay;
uint16_t grain2PhaseAcc;
uint16_t grain2PhaseInc;
uint16_t grain2Amp;
uint8_t grain2Decay;

// Map Analogue channels
#define SYNC_CONTROL         (4)
#define GRAIN_FREQ_CONTROL   (0)
#define GRAIN_DECAY_CONTROL  (2)
#define GRAIN2_FREQ_CONTROL  (3)
#define GRAIN2_DECAY_CONTROL (1)


// Changing these will also requires rewriting audioOn()

#if defined(__AVR_ATmega8__)
//
// On old ATmega8 boards.
//    Output is on pin 11
//
#define LED_PIN       13
#define LED_PORT      PORTB
#define LED_BIT       5
#define PWM_PIN       11
#define PWM_VALUE     OCR2
#define PWM_INTERRUPT TIMER2_OVF_vect
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
//
// On the Arduino Mega
//    Output is on pin 3
//
#define LED_PIN       3
#define LED_PORT      PORTB
#define LED_BIT       7
#define PWM_PIN       13
#define PWM_VALUE     OCR3C
#define PWM_INTERRUPT TIMER3_OVF_vect
#else
//
// For modern ATmega168 and ATmega328 boards
//    Output is on pin 3
//
#define PWM_PIN       3
#define PWM_VALUE     OCR2B
#define LED_PIN       13
#define LED_PORT      PORTB
#define LED_BIT       5
#define PWM_INTERRUPT TIMER2_OVF_vect
#endif

// Smooth logarithmic mapping
//
uint16_t antilogTable[] = {
  64830,64132,63441,62757,62081,61413,60751,60097,59449,58809,58176,57549,56929,56316,55709,55109,
  54515,53928,53347,52773,52204,51642,51085,50535,49991,49452,48920,48393,47871,47356,46846,46341,
  45842,45348,44859,44376,43898,43425,42958,42495,42037,41584,41136,40693,40255,39821,39392,38968,
  38548,38133,37722,37316,36914,36516,36123,35734,35349,34968,34591,34219,33850,33486,33125,32768
};
uint16_t mapPhaseInc(uint16_t input) {
  return (antilogTable[input & 0x3f]) >> (input >> 6);
}

// Stepped chromatic mapping
//
uint16_t midiTable[] = {
  17,18,19,20,22,23,24,26,27,29,31,32,34,36,38,41,43,46,48,51,54,58,61,65,69,73,
  77,82,86,92,97,103,109,115,122,129,137,145,154,163,173,183,194,206,218,231,
  244,259,274,291,308,326,346,366,388,411,435,461,489,518,549,581,616,652,691,
  732,776,822,871,923,978,1036,1097,1163,1232,1305,1383,1465,1552,1644,1742,
  1845,1955,2071,2195,2325,2463,2610,2765,2930,3104,3288,3484,3691,3910,4143,
  4389,4650,4927,5220,5530,5859,6207,6577,6968,7382,7821,8286,8779,9301,9854,
  10440,11060,11718,12415,13153,13935,14764,15642,16572,17557,18601,19708,20879,
  22121,23436,24830,26306
};
uint16_t mapMidi(uint16_t input) {
  return (midiTable[(1023-input) >> 3]);
}

// Stepped Pentatonic mapping
//
uint16_t pentatonicTable[54] = {
  0,19,22,26,29,32,38,43,51,58,65,77,86,103,115,129,154,173,206,231,259,308,346,
  411,461,518,616,691,822,923,1036,1232,1383,1644,1845,2071,2463,2765,3288,
  3691,4143,4927,5530,6577,7382,8286,9854,11060,13153,14764,16572,19708,22121,26306
};

uint16_t mapPentatonic(uint16_t input) {
  uint8_t value = (1023-input) / (1024/53);
  return (pentatonicTable[value]);
}


void audioOn() {
#if defined(__AVR_ATmega8__)
  // ATmega8 has different registers
  TCCR2 = _BV(WGM20) | _BV(COM21) | _BV(CS20);
  TIMSK = _BV(TOIE2);
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  TCCR3A = _BV(COM3C1) | _BV(WGM30);
  TCCR3B = _BV(CS30);
  TIMSK3 = _BV(TOIE3);
#else
  // Set up PWM to 31.25kHz, phase accurate
  TCCR2A = _BV(COM2B1) | _BV(WGM20);
  TCCR2B = _BV(CS20);
  TIMSK2 = _BV(TOIE2);
#endif
}

int current_tempo = 120;
int previous_tempo = 120;
int pattern = 0;
int counter = 0;
long ms_clock = 0;
long prev_tmstmp = 0;
// long bpm = 120;
// long tempo = 1000/(bpm/60);
// long prevmillis = 0;
// long interval = tempo/24;    //interval is the number of milliseconds defined by tempo formula.


bool int_clock = false;
bool ext_clock = false;
bool chosen_clock = false;

int a1 = 0; int a2 = 0; int a3 = 0; int a4 = 0; int a5 = 0; 
int b1 = 0; int b2 = 0; int b3 = 0; int b4 = 0; int b5 = 0; 
int c1 = 0; int c2 = 0; int c3 = 0; int c4 = 0; int c5 = 0; 
int d1 = 0; int d2 = 0; int d3 = 0; int d4 = 0; int d5 = 0; 
int e1 = 0; int e2 = 0; int e3 = 0; int e4 = 0; int e5 = 0; 
int f1 = 0; int f2 = 0; int f3 = 0; int f4 = 0; int f5 = 0; 
int g1 = 0; int g2 = 0; int g3 = 0; int g4 = 0; int g5 = 0; 
int h1 = 0; int h2 = 0; int h3 = 0; int h4 = 0; int h5 = 0; 
int i1 = 0; int i2 = 0; int i3 = 0; int i4 = 0; int i5 = 0; 
int j1 = 0; int j2 = 0; int j3 = 0; int j4 = 0; int j5 = 0; 
int k1 = 0; int k2 = 0; int k3 = 0; int k4 = 0; int k5 = 0; 
int l1 = 0; int l2 = 0; int l3 = 0; int l4 = 0; int l5 = 0; 
int m1 = 0; int m2 = 0; int m3 = 0; int m4 = 0; int m5 = 0; 
int n1 = 0; int n2 = 0; int n3 = 0; int n4 = 0; int n5 = 0; 
int o1 = 0; int o2 = 0; int o3 = 0; int o4 = 0; int o5 = 0; 
int p1 = 0; int p2 = 0; int p3 = 0; int p4 = 0; int p5 = 0;

int live_sync_phase = 0;
int live_grain_phase = 0;
int live_grain_decay = 0;
int live_grain2_phase = 0;
int live_grain2_decay = 0;

#define MAX_DELAY 2048
unsigned char sDelayBuffer[MAX_DELAY];
unsigned int nDelayCounter = 0;
unsigned char bDelay;

int current_steps = 8;
int previous_steps = 8;

LiquidCrystal lcd(1, 2, 4, 5, 6, 7);


//BUTTON MANAGEMENT
#define DEBOUNCE 2  // button debouncer, how many ms to debounce, 5+ ms is usually plenty
// here is where we define the buttons that we'll use. button "1" is the first, button "6" is the 6th, etc
byte buttons[] = {37,35,33}; // the analog 0-5 pins are also known as 14-19
// This handy macro lets us determine how big the array up above is, by checking the size
#define NUMBUTTONS sizeof(buttons)
// we will track if a button is just pressed, just released, or 'currently pressed' 
byte pressed[NUMBUTTONS], justpressed[NUMBUTTONS], justreleased[NUMBUTTONS];

void check_switches()
{
  static byte previousstate[NUMBUTTONS];
  static byte currentstate[NUMBUTTONS];
  static long lasttime;
  byte index;

  if (millis() < lasttime){ // we wrapped around, lets just try again
     lasttime = millis();
  }
  
  if ((lasttime + DEBOUNCE) > millis()) {
    // not enough time has passed to debounce
    return; 
  }
  // ok we have waited DEBOUNCE milliseconds, lets reset the timer
  lasttime = millis();
  
  for (index = 0; index < NUMBUTTONS; index++){ // when we start, we clear out the "just" indicators
    justpressed[index] = 0;
    justreleased[index] = 0; 
    currentstate[index] = digitalRead(buttons[index]);   // read the button
     
    if (currentstate[index] == previousstate[index]) {
      if ((pressed[index] == LOW) && (currentstate[index] == LOW)) {
          // just pressed
          justpressed[index] = 1;
      }
      else if ((pressed[index] == HIGH) && (currentstate[index] == HIGH)) {
          // just released
          justreleased[index] = 1;
      }
      pressed[index] = !currentstate[index];  // remember, digital HIGH means NOT pressed
    }
    //Serial.println(pressed[index], DEC);
    previousstate[index] = currentstate[index];   // keep a running tally of the buttons
  }
}

void setup() {

  lcd.begin(16,2);

  pinMode(PWM_PIN,OUTPUT);
  audioOn();
  pinMode(LED_PIN,OUTPUT);
  
  //FIRST ROW 8 LED'S
  pinMode(39, OUTPUT); digitalWrite(39, LOW);
  pinMode(41, OUTPUT); digitalWrite(41, LOW);
  pinMode(43, OUTPUT); digitalWrite(43, LOW);
  pinMode(45, OUTPUT); digitalWrite(45, LOW);
  pinMode(47, OUTPUT); digitalWrite(47, LOW);
  pinMode(49, OUTPUT); digitalWrite(49, LOW);
  pinMode(51, OUTPUT); digitalWrite(51, LOW);
  pinMode(53, OUTPUT); digitalWrite(53, LOW);
  //SECOND ROW 8 LED'S
  pinMode(38, OUTPUT); digitalWrite(38, LOW);
  pinMode(40, OUTPUT); digitalWrite(40, LOW);
  pinMode(42, OUTPUT); digitalWrite(42, LOW);
  pinMode(44, OUTPUT); digitalWrite(44, LOW);
  pinMode(46, OUTPUT); digitalWrite(46, LOW);
  pinMode(48, OUTPUT); digitalWrite(48, LOW);
  pinMode(50, OUTPUT); digitalWrite(50, LOW);
  pinMode(52, OUTPUT); digitalWrite(52, LOW);

  //SECOND ROW BUTTONS
  pinMode(22, INPUT_PULLUP); digitalWrite(22, HIGH);
  pinMode(24, INPUT_PULLUP); digitalWrite(24, HIGH);
  pinMode(26, INPUT_PULLUP); digitalWrite(26, HIGH);
  pinMode(28, INPUT_PULLUP); digitalWrite(28, HIGH);

  //FIRST ROW BUTTONS
  pinMode(30, INPUT_PULLUP); digitalWrite(30, HIGH);
  pinMode(32, INPUT_PULLUP); digitalWrite(32, HIGH);
  pinMode(34, INPUT_PULLUP); digitalWrite(34, HIGH);
  pinMode(36, INPUT_PULLUP); digitalWrite(36, HIGH);

  //THIRD ROW BUTTONS
  pinMode(37, INPUT_PULLUP); digitalWrite(37, HIGH);
  pinMode(35, INPUT_PULLUP); digitalWrite(37, HIGH);
  pinMode(33, INPUT_PULLUP); digitalWrite(35, HIGH);

  //SWITCHES
  pinMode(31, INPUT); digitalWrite(31, HIGH);
  pinMode(29, INPUT); digitalWrite(29, HIGH);
  pinMode(27, INPUT); digitalWrite(27, HIGH);
 

}


void changeStep(int step_num){

/* The first thing we do is to turn off all indicator lights so that we can properly indicate 
which step we're currently editing. */  
  
  digitalWrite(39, LOW);digitalWrite(41, LOW);digitalWrite(43, LOW);digitalWrite(45, LOW);
  digitalWrite(47, LOW);digitalWrite(49, LOW);digitalWrite(51, LOW);digitalWrite(53, LOW);
  digitalWrite(38, LOW);digitalWrite(40, LOW);digitalWrite(42, LOW);digitalWrite(44, LOW);
  digitalWrite(46, LOW);digitalWrite(48, LOW);digitalWrite(50, LOW);digitalWrite(52, LOW); 

// Then indicate the appropriate step.  

    switch(step_num){
    
    case 1:
    digitalWrite(53, HIGH); break;
    case 2:
    digitalWrite(51, HIGH); break;
    case 3:
    digitalWrite(49, HIGH); break;
    case 4:
    digitalWrite(47, HIGH); break;
    case 5:
    digitalWrite(45, HIGH); break;
    case 6:
    digitalWrite(43, HIGH); break;
    case 7:
    digitalWrite(41, HIGH); break; 
    case 8:
    digitalWrite(39, HIGH); break;
    case 9:
    digitalWrite(52, HIGH); break;
    case 10:
    digitalWrite(50, HIGH); break;
    case 11:
    digitalWrite(48, HIGH); break;
    case 12:
    digitalWrite(46, HIGH); break;
    case 13:
    digitalWrite(44, HIGH); break;
    case 14:
    digitalWrite(42, HIGH); break;
    case 15:
    digitalWrite(40, HIGH); break; 
    case 16:
    digitalWrite(38, HIGH); break;
  }

/* This next chunk of code is fairly similar to the unaltered Auduino sketch. This allows 
us to continue updating the synth parameters to the user input. That way, you can dial in 
the sound of a particular step. The while-loop traps the program flow here until the user 
pushes button 1. As the code currently stands, "live" parameters aren't applied while in 
the step editor but you could easily add the live parameters below. */

while(1){  
  // lcd.clear(); 
  // lcd.print(step_num);
  // counter++;
  // if(counter>tempo){
  
  counter=0;
  syncPhaseInc = mapPentatonic(analogRead(SYNC_CONTROL));
  grainPhaseInc  = mapPhaseInc(analogRead(GRAIN_FREQ_CONTROL)) / 2;
  grainDecay     = analogRead(GRAIN_DECAY_CONTROL) / 8;
  grain2PhaseInc = mapPhaseInc(analogRead(GRAIN2_FREQ_CONTROL)) / 2;
  grain2Decay    = analogRead(GRAIN2_DECAY_CONTROL) / 4; 

//Here we read the button 1 input and commit the step changes to the appropriate parameters.
  
  if(digitalRead(37)==LOW && step_num==1){
  a1 = syncPhaseInc; a2 = grainPhaseInc; a3 = grainDecay; a4 = grain2PhaseInc; a5 = grain2Decay; return;}
  else if(digitalRead(37)==LOW && step_num==2){
  b1 = syncPhaseInc; b2 = grainPhaseInc; b3 = grainDecay; b4 = grain2PhaseInc; b5 = grain2Decay; return;}
  else if(digitalRead(37)==LOW && step_num==3){
  c1 = syncPhaseInc; c2 = grainPhaseInc; c3 = grainDecay; c4 = grain2PhaseInc; c5 = grain2Decay; return;}
  else if(digitalRead(37)==LOW && step_num==4){
  d1 = syncPhaseInc; d2 = grainPhaseInc; d3 = grainDecay; d4 = grain2PhaseInc; d5 = grain2Decay; return;}
  else if(digitalRead(37)==LOW && step_num==5){
  e1 = syncPhaseInc; e2 = grainPhaseInc; e3 = grainDecay; e4 = grain2PhaseInc; e5 = grain2Decay; return;}
  else if(digitalRead(37)==LOW && step_num==6){
  f1 = syncPhaseInc; f2 = grainPhaseInc; f3 = grainDecay; f4 = grain2PhaseInc; f5 = grain2Decay; return;}
  else if(digitalRead(37)==LOW && step_num==7){
  g1 = syncPhaseInc; g2 = grainPhaseInc; g3 = grainDecay; g4 = grain2PhaseInc; g5 = grain2Decay; return;}
  else if(digitalRead(37)==LOW && step_num==8){
  h1 = syncPhaseInc; h2 = grainPhaseInc; h3 = grainDecay; h4 = grain2PhaseInc; h5 = grain2Decay; return;}
  else if(digitalRead(37)==LOW && step_num==9){
  i1 = syncPhaseInc; i2 = grainPhaseInc; i3 = grainDecay; i4 = grain2PhaseInc; i5 = grain2Decay; return;}
  else if(digitalRead(37)==LOW && step_num==10){
  j1 = syncPhaseInc; j2 = grainPhaseInc; j3 = grainDecay; j4 = grain2PhaseInc; j5 = grain2Decay; return;}
  else if(digitalRead(37)==LOW && step_num==11){
  k1 = syncPhaseInc; k2 = grainPhaseInc; k3 = grainDecay; k4 = grain2PhaseInc; k5 = grain2Decay; return;}
  else if(digitalRead(37)==LOW && step_num==12){
  l1 = syncPhaseInc; l2 = grainPhaseInc; l3 = grainDecay; l4 = grain2PhaseInc; l5 = grain2Decay; return;}
  else if(digitalRead(37)==LOW && step_num==13){
  m1 = syncPhaseInc; m2 = grainPhaseInc; m3 = grainDecay; m4 = grain2PhaseInc; m5 = grain2Decay; return;}
  else if(digitalRead(37)==LOW && step_num==14){
  n1 = syncPhaseInc; n2 = grainPhaseInc; n3 = grainDecay; n4 = grain2PhaseInc; n5 = grain2Decay; return;}
  else if(digitalRead(37)==LOW && step_num==15){
  o1 = syncPhaseInc; o2 = grainPhaseInc; o3 = grainDecay; o4 = grain2PhaseInc; o5 = grain2Decay; return;}
  else if(digitalRead(37)==LOW && step_num==16){
  p1 = syncPhaseInc; p2 = grainPhaseInc; p3 = grainDecay; p4 = grain2PhaseInc; p5 = grain2Decay; return;}

}}

void loop() {

  check_switches();

  //HOLD SECOND AND THIRD SHIFT FOR ADJUST NRS OF STEPS
  if(pressed[1] && pressed[2]){
    current_steps = map(analogRead(15),0,1023,1,17);
    if(previous_steps != current_steps){
      lcd.clear(); 
      lcd.print(current_steps);
      pattern = 0;
    }
    previous_steps = current_steps;
    
  }

  //HOLD SECOND SHIFT FOR ADJUST TEMPO
  if(pressed[1] && !pressed[2]){
    current_tempo = map(analogRead(15),0,1023,60,180);
    
    if(previous_tempo != current_tempo){
      lcd.clear(); 
      lcd.print(current_tempo);
    }
    previous_tempo = current_tempo;
  }
  
  ms_clock = ((60000/current_tempo)/4); // 1/16 steps
  unsigned long current_tmpstmp = millis();


  if(digitalRead(27)==HIGH){
    if(current_tmpstmp - prev_tmstmp > ms_clock){
      prev_tmstmp = current_tmpstmp;
      int_clock = true;
    }
    else{
      int_clock = false;
    }
    chosen_clock = int_clock;  
  }
  // else{
  //   if(digitalRead(11)==1){
  //     ext_clock = true;
  //     unsigned long diff_tmpstmp = current_tmpstmp - prev_tmstmp;
  //     prev_tmstmp = current_tmpstmp;
  //     tempo = (60000/diff_tmpstmp)/2;
  //     lcd.clear(); 
  //     lcd.print(tempo);
  //     }
  //   else{
  //     ext_clock = false;
  //   }
  //   chosen_clock = ext_clock;
  // }
  
  counter++;

/* Most of the time, the main loop will just advance the counter while we continue generating noise. 
Each iteration, we check the counter against our "tempo" parameter to find out if it's time yet to 
jump to the next step. */  
  
  if(chosen_clock){
    delay(20);
  //Tempo Control for int_clock
 
  //Delay for ext_clock
  //delay(20);
 
//Housecleaning: Just a few things to get out of the way since the counter is "full"
  counter=0;
  if(pattern==current_steps){pattern=0;}
  pattern++;
  digitalWrite(39, LOW);digitalWrite(41, LOW);digitalWrite(43, LOW);digitalWrite(45, LOW);
  digitalWrite(47, LOW);digitalWrite(49, LOW);digitalWrite(51, LOW);digitalWrite(53, LOW);
  digitalWrite(38, LOW);digitalWrite(40, LOW);digitalWrite(42, LOW);digitalWrite(44, LOW);
  digitalWrite(46, LOW);digitalWrite(48, LOW);digitalWrite(50, LOW);digitalWrite(52, LOW);
  
 
//Live Tweaks: Read the analog inputs associated with each "live" parameter.
  live_sync_phase = map(analogRead(14),0,1023,-500,500);
  live_grain_phase = map(analogRead(10),0,1023,-200,200);
  live_grain_decay = map(analogRead(9),0,1023,-20,20);
  live_grain2_phase = map(analogRead(8),0,1023,-200,200);
  live_grain2_decay = map(analogRead(11),0,1023,-50,50);
  
//Grab the parameters for the step that we're now in. We'll use a series of case
//statements switched on the "pattern" variable that we incremented earlier.

/* In each of the case routines below you'll notice that we're addressing 
each of the existing Auduino parameters and making them equal to the stored 
parameter plus the associated "live" parameter. */
  if(digitalRead(31) == HIGH){
    switch(pattern){
      case 1:
      syncPhaseInc = a1 + live_sync_phase; grainPhaseInc = a2 + live_grain_phase; 
      grainDecay = a3 + live_grain_decay; grain2PhaseInc = a4 + live_grain2_phase;
      grain2Decay = a5 + live_grain2_decay; 
      digitalWrite(53, HIGH); break;
      case 2:
      syncPhaseInc = b1 + live_sync_phase; grainPhaseInc = b2 + live_grain_phase; 
      grainDecay = b3 + live_grain_decay; grain2PhaseInc = b4 + live_grain2_phase; 
      grain2Decay = b5 + live_grain2_decay; 
      digitalWrite(51, HIGH); break;
      case 3:
      syncPhaseInc = c1 + live_sync_phase; grainPhaseInc = c2 + live_grain_phase; 
      grainDecay = c3 + live_grain_decay; grain2PhaseInc = c4 + live_grain2_phase; 
      grain2Decay = c5 + live_grain2_decay; 
      digitalWrite(49, HIGH); break;
      case 4:
      syncPhaseInc = d1 + live_sync_phase; grainPhaseInc = d2 + live_grain_phase; 
      grainDecay = d3 + live_grain_decay; grain2PhaseInc = d4 + live_grain2_phase; 
      grain2Decay = d5 + live_grain2_decay; 
      digitalWrite(47, HIGH); break;
      case 5:
      syncPhaseInc = e1 + live_sync_phase; grainPhaseInc = e2 + live_grain_phase; 
      grainDecay = e3 + live_grain_decay; grain2PhaseInc = e4 + live_grain2_phase;
      grain2Decay = e5 + live_grain2_decay; 
      digitalWrite(45, HIGH); break;
      case 6:
      syncPhaseInc = f1 + live_sync_phase; grainPhaseInc = f2 + live_grain_phase; 
      grainDecay = f3 + live_grain_decay; grain2PhaseInc = f4 + live_grain2_phase; 
      grain2Decay = f5 + live_grain2_decay; 
      digitalWrite(43, HIGH); break;
      case 7:
      syncPhaseInc = g1 + live_sync_phase; grainPhaseInc = g2 + live_grain_phase; 
      grainDecay = g3 + live_grain_decay; grain2PhaseInc = g4 + live_grain2_phase;
      grain2Decay = g5 + live_grain2_decay; 
      digitalWrite(41, HIGH); break; 
      case 8:
      syncPhaseInc = h1 + live_sync_phase; grainPhaseInc = h2 + live_grain_phase; 
      grainDecay = h3 + live_grain_decay; grain2PhaseInc = h4 + live_grain2_phase; 
      grain2Decay = h5 + live_grain2_decay; 
      digitalWrite(39, HIGH); break;
    }
  }else{
    switch(pattern){
      case 1:
      syncPhaseInc = a1; grainPhaseInc = a2; grainDecay = a3; grain2PhaseInc = a4; grain2Decay = a5; 
      digitalWrite(53, HIGH); break;
      case 2:
      syncPhaseInc = b1; grainPhaseInc = b2; grainDecay = b3; grain2PhaseInc = b4; grain2Decay = b5; 
      digitalWrite(51, HIGH); break;
      case 3:
      syncPhaseInc = c1; grainPhaseInc = c2; grainDecay = c3; grain2PhaseInc = c4; grain2Decay = c5;
      digitalWrite(49, HIGH); break;
      case 4:
      syncPhaseInc = d1; grainPhaseInc = d2; grainDecay = d3; grain2PhaseInc = d4; grain2Decay = d5;
      digitalWrite(47, HIGH); break;
      case 5:
      syncPhaseInc = e1; grainPhaseInc = e2; grainDecay = e3; grain2PhaseInc = e4; grain2Decay = e5;
      digitalWrite(45, HIGH); break;
      case 6:
      syncPhaseInc = f1; grainPhaseInc = f2; grainDecay = f3; grain2PhaseInc = f4; grain2Decay = f5; 
      digitalWrite(43, HIGH); break;
      case 7:
      syncPhaseInc = g1; grainPhaseInc = g2; grainDecay = g3; grain2PhaseInc = g4; grain2Decay = g5;
      digitalWrite(41, HIGH); break; 
      case 8:
      syncPhaseInc = h1; grainPhaseInc = h2; grainDecay = h3; grain2PhaseInc = h4; grain2Decay = h5;
      digitalWrite(39, HIGH); break;
      case 9:
      syncPhaseInc = i1; grainPhaseInc = i2; grainDecay = i3; grain2PhaseInc = i4; grain2Decay = i5; 
      digitalWrite(52,HIGH); break;
      case 10:
      syncPhaseInc = j1; grainPhaseInc = j2; grainDecay = j3; grain2PhaseInc = j4; grain2Decay = j5; 
      digitalWrite(50,HIGH); break;
      case 11:
      syncPhaseInc = k1; grainPhaseInc = k2; grainDecay = k3; grain2PhaseInc = k4; grain2Decay = k5;
      digitalWrite(48,HIGH); break;
      case 12:
      syncPhaseInc = l1; grainPhaseInc = l2; grainDecay = l3; grain2PhaseInc = l4; grain2Decay = l5;
      digitalWrite(46,HIGH); break;
      case 13:
      syncPhaseInc = m1; grainPhaseInc = m2; grainDecay = m3; grain2PhaseInc = m4; grain2Decay = m5;
      digitalWrite(44,HIGH); break;
      case 14:
      syncPhaseInc = n1; grainPhaseInc = n2; grainDecay = n3; grain2PhaseInc = n4; grain2Decay = n5; 
      digitalWrite(42,HIGH); break;
      case 15:
      syncPhaseInc = o1; grainPhaseInc = o2; grainDecay = o3; grain2PhaseInc = o4; grain2Decay = o5;
      digitalWrite(40,HIGH); break; 
      case 16:
      syncPhaseInc = p1; grainPhaseInc = p2; grainDecay = p3; grain2PhaseInc = p4; grain2Decay = p5;
      digitalWrite(38,HIGH); break;
    }
  }
//Check to see if the user is trying to change the step parameters.
//This series of statements simply check for a button press from each of
//the step buttons and call a function to change the indicated step.    

    if(digitalRead(29) == LOW){
      if(digitalRead(30)==LOW){changeStep(1);}
      if(digitalRead(32)==LOW){changeStep(2);}
      if(digitalRead(34)==LOW){changeStep(3);}
      if(digitalRead(36)==LOW){changeStep(4);}
      if(digitalRead(22)==LOW){changeStep(5);}
      if(digitalRead(24)==LOW){changeStep(6);}
      if(digitalRead(26)==LOW){changeStep(7);}
      if(digitalRead(28)==LOW){changeStep(8);}
    }
    else if(digitalRead(29) == HIGH){
      if(digitalRead(30)==LOW){changeStep(9);}
      if(digitalRead(32)==LOW){changeStep(10);}
      if(digitalRead(34)==LOW){changeStep(11);}
      if(digitalRead(36)==LOW){changeStep(12);}
      if(digitalRead(22)==LOW){changeStep(13);}
      if(digitalRead(24)==LOW){changeStep(14);}
      if(digitalRead(26)==LOW){changeStep(15);}
      if(digitalRead(28)==LOW){changeStep(16);}
    }

    


}}

SIGNAL(PWM_INTERRUPT)
{
  uint8_t value;
  uint16_t output;

  syncPhaseAcc += syncPhaseInc;
  if (syncPhaseAcc < syncPhaseInc) {
    // Time to start the next grain
    grainPhaseAcc = 0;
    grainAmp = 0x7fff;
    grain2PhaseAcc = 0;
    grain2Amp = 0x7fff;
    LED_PORT ^= 1 << LED_BIT; // Faster than using digitalWrite
  }
  
  // Increment the phase of the grain oscillators
  grainPhaseAcc += grainPhaseInc;
  grain2PhaseAcc += grain2PhaseInc;

  // Convert phase into a triangle wave
  value = (grainPhaseAcc >> 7) & 0xff;
  if (grainPhaseAcc & 0x8000) value = ~value;
  // Multiply by current grain amplitude to get sample
  output = value * (grainAmp >> 8);

  // Repeat for second grain
  value = (grain2PhaseAcc >> 7) & 0xff;
  if (grain2PhaseAcc & 0x8000) value = ~value;
  output += value * (grain2Amp >> 8);

  // Make the grain amplitudes decay by a factor every sample (exponential decay)
  grainAmp -= (grainAmp >> 8) * grainDecay;
  grain2Amp -= (grain2Amp >> 8) * grain2Decay;

  // Scale output to the available range, clipping if necessary
  output >>= 9;
  if (output > 255) output = 255;

  if(pressed[0] && pressed[1]){
    // MAX_DELAY = map(analogRead(14),0,1023,512,500);
    // Output to PWM (this is faster than using analogWrite) 
    // Here we add the delay buffer to the output value, this produces
    // an subtle echo effect, the delay buffer is effectivley replaying the sound from
    // 1/8th of a second ago.
 
    LED_PORT |= 1 << LED_BIT; // Faster than using digitalWrite
    PWM_VALUE = (output + (sDelayBuffer[nDelayCounter]))>>1;
   
    // add the new output to the buffer so we can use it when the buffer next wraps around
    sDelayBuffer[nDelayCounter] = PWM_VALUE;
    nDelayCounter++;
    if(nDelayCounter == MAX_DELAY){
      nDelayCounter = 0;
    }
  }
  else{
    LED_PORT &= ~(1 << LED_BIT); // Faster than using digitalWrite
    PWM_VALUE = output;
  }

  // Output to PWM (this is faster than using analogWrite)  
  // PWM_VALUE = output;
}
