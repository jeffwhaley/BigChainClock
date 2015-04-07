/*
  Big Chain Clock
  Jeff Whaley
  Created: 17 Nov 2012
  History (Not Complete):
  19 Jan 2013 - left this program running for first time, to see if stays close to in time, bell buttons enabled
  26 Jan 2013 - cranked through the To Do list, partially got quarters and hour chimes working...
  9 Mar 2013 - new approach on fast forward, table based instead of algorithm based
  27 July 2013 - fixed the issue where if you press all buttons to set midnight the clock moves ahead a little
        found an overrun case where serial print is taking time
        also seems like the issue with getting stuck on quarter hours is mechanical due to the numbers whipping around at bottom of chain, slowed it down a little and ran ff a long time, seems to work now 
  
  To Do
  Handle usec overflow correctly for task start - Done 1/26
  Move Hour count function into min step task and handle the non-integer math - Done 1/26
  Chime out the hours and use the Bell Off switch
  Get the fast forward working - Done
  Do the set function - Done 1/26
  Get the reverse function working 7/27!
  Get 3rd bell working - Done 1/26, can't suse pins 0/1 for digital output because they are serial port duh!
  
  The control program for the Big Chain Clock installed at Chicago Avenue Fire Arts Center in Minneapolis, MN. http://www.cafac.org
  For more info see blog at: http://bigchainclock.blogspot.com/
  
  Button functions:
  Red buttons ring the bells on transition from not depressed to depressed
  Switch on side enables or disables bells
  Black up botton accelerates clock when depressed, clock slows to normal rate when not depressed
  When both black buttons are depressed clock stops
  Black down button deccelerates clock when depressed including going backwards
  When all 5 buttons are pressed time resets to 12:00, used for syncing clock if power lost
  
  Basic task concept is that each enabled task has a time when it will be run next
  
  Task limitations: 
  No long delays anywhere in program to prevent tasks from being held off, lets say 10 usec max, but its really dependent on shortest task interval, which is currently 35 ms for minute stepper
  Longest time a task can be set to run in future is ~35 min, based on 32 bit usec overflow of ~71 min / 2.
   
  for each task
    if taskenabled && ctime > taskruntime then run task
    
  task template
    do something useful
    set or clear taskenabled
    if taskenabled set then set taskruntime (usually taskruntime+= something)
    
  tasks:
    keeptime - moves the chains, for normal time keeping call at minute step rate interval
    bell1, 2, 3off - turns bell driver off
    bell1, 2, 3on - turns bell driver on
    
  Functionality Plan
  1
    Clock runs at correct speed
    Bell buttons work
  2
    Bell off switch works
    Bells chime simple pattern
    Clock set process works - off, move chain to 1200, start
  3
    Fast Forward button works
    Stop button works
  4
    Reverse button works
    Clock set process works - use buttons to set to 1200, press all 

  Fast Forward Design
  need to add a variable step time to keeptime task, the timeset task will change it up or down to get time set, then change it back to calibrated normal steptime MSTEPTIME
  if ffbutton pressed enable timeset and turn off bells (check if timeseten is false before ringing bells)
  at time intervals FFCHECKTIME check the ffbutton if still pressed decrease step time by a factor <1 FFACCEL until it reaches a limit  FFLIMIT, then don't change, this is fastest clock will go
  if ffbutton no longer pressed increase step time by a factor FFDECELQ10 >1 until it gets back to MSTEPTIME, then disable timeset
    
*/

// Arduino Pin Definitions
const int clkdirpin = 13;
const int clkenablepin = 11;
const int msteppin = 10;
const int hsteppin = 9;
const int clkresetpin = 12;
const int bell1pinA = 2;
const int bell1pinB = 3;
const int bell2pinA = 6;
const int bell2pinB = 7;
const int bell3pinA = 4;
const int bell3pinB = 5;
const int B1BTNPIN = A0;
const int B2BTNPIN = A1;
const int B3BTNPIN = A2;
const int FFBTNPIN = A4;
const int REVBTNPIN = A3;
const int BSWITCHPIN = A5;

const unsigned long int N0PT95Q16 = 62259UL;
const unsigned long int N1PT05Q16 = 68813UL;
const unsigned long int N1PT0Q24 = 16777216UL;
const unsigned long int N1PT0Q30 = 1073741824UL;
const unsigned long int HALFMAXUL = 2147483648UL;

const unsigned long int H2MQ24 = 2030184UL; // Q24 ratio of min steps to hour steps 
const unsigned long int BELLTIME = 40000UL; // time in usec between turning on bell driver and turning it off
const long int MSTEPTIME = 34981L; // time between minute steps in usec
const long int STEPSPERMINQ30 = 626457L; // usec per min

//constants related to setting time
const unsigned long int TSCHECKTIME = 100000UL; // time interval when ff button is checked and speed adjusted in usec
const int TSSTEPS[] = {3180, 1666, 1128, 853, 686, 573, 493, 432, 384};
const int NUMTSSTEPS = 7;

const unsigned long int CHIMETIME = 2000000UL; //time between hour chimes in us

// globals
long unsigned mstepcount = 0, hstepcount = 0, hstepcountQ24 = 0;
unsigned long msteptime; // current min step time in usec, set to MSTEPTIME when in normal clock time keeping mode
int hours = 12, minutes = 0;  // hours and min in human readable time
long unsigned minQ30 = 0; // minutes in Q30, incremented in min step function

unsigned long int ctime, ctimeprev; //current and prev time in usec
boolean ffbutton, revbutton, bell1button, bell2button, bell3button, bellswitch;  // boolean variables for button and switch state
boolean ffbuttonprev, revbuttonprev, bell1buttonprev, bell2buttonprev, bell3buttonprev;  // boolean variables for previous button state

boolean normaldirection; // true if time going forward, false if backward

//task variables
//enable bits
boolean keeptimeen = false, bell1offen = false, bell2offen = false, bell3offen = false, bell1onen = false, bell2onen = false, bell3onen = false, 
  chimehoursen = false, timeseten =false;

// task run times
unsigned long int keeptimert, bell1offrt, bell2offrt, bell3offrt, bell1onrt, bell2onrt, bell3onrt,
  chimehoursrt, timesetrt;
  
// task states (optional)
int chimehoursstate;
const int START = 1, CHIMEHOURS = 6;  // use to tell state

int tsindex = 0; // for timeset - index into timestep table 

int cqhours; // for chimehours - what to chime

//tasks
void keeptime() {
  mstep(); // step the minute stepper motor and the hour stepper motor if needed
  keeptimert += msteptime; // set next interrupt, usec resolution on step time, might need more accurate way to do this to keep clock accurate enough, time will tell...
  if (normaldirection) {
    // keep track of what time it is
    minQ30 += STEPSPERMINQ30;
    if (minQ30 > N1PT0Q30) {
      minQ30 -= N1PT0Q30;
      minutes++;
      minutes %= 60;
      if (minutes == 0) {
        hours++;
        if (hours == 13) hours = 1;
      }
      if (minutes == 0 && bellswitch)  { //start chimes every hour unless bell switch is on
        cqhours = hours;
        chimehoursen = true;
        chimehoursrt = ctime;
        chimehoursstate = START;
      } 
    }
  } else { // going backward    
    // keep track of what time it is
    if (minQ30 >= STEPSPERMINQ30) {
      minQ30 -= STEPSPERMINQ30;
    } else { // went below 0 subtract a min
      minQ30 += N1PT0Q30 - STEPSPERMINQ30;
      minutes--;
      if (minutes == -1) {
        minutes = 59;
        hours--;
        if (hours == 0) hours = 12;
      }
      if (minutes == 59 && bellswitch)  { //start chimes every hour unless bell switch is on
        if (hours == 12) cqhours = 1; else cqhours = hours + 1;
        chimehoursen = true;
        chimehoursrt = ctime;
        chimehoursstate = START;
      }
    }
  }
    // Serial.print("Time is:");
    // Serial.print(hours);
    // Serial.print(":");  if (minutes<10) Serial.print("0");
    // Serial.println(minutes);  
}

void bell1on() {
  digitalWrite(bell1pinA, HIGH);  
  digitalWrite(bell1pinB, LOW);
  bell1onen = false;
  bell1offen = true; //enable bell off task
  bell1offrt = micros() + BELLTIME; //set bell to turn off in BELLTIME us
}

void bell1off() {
  digitalWrite(bell1pinA, LOW);
  digitalWrite(bell1pinB, LOW);
  bell1offen = false; //disable bell off task
}

void bell2on() {
  digitalWrite(bell2pinA, HIGH);  
  digitalWrite(bell2pinB, LOW);
  bell2onen = false;
  bell2offen = true; //enable bell off task
  bell2offrt = micros() + BELLTIME; //set bell to turn off in BELLTIME us
}

void bell2off() {
  digitalWrite(bell2pinA, LOW);
  digitalWrite(bell2pinB, LOW);
  bell2offen = false; //disable bell off task
}

void bell3on() {
  digitalWrite(bell3pinA, HIGH);  
  digitalWrite(bell3pinB, LOW);
  bell3onen = false;
  bell3offen = true; //enable bell off task
  bell3offrt = micros() + BELLTIME; //set bell to turn off in BELLTIME us
}

void bell3off() {
  digitalWrite(bell3pinA, LOW);
  digitalWrite(bell3pinB, LOW);
  bell3offen = false; //disable bell off task
}

void chimehours() { // uses globals cqhours to chime the right time
  static int cqcount;
  if (chimehoursstate == START) {
    cqcount = 0;
    chimehoursstate = CHIMEHOURS; 
  } 
  if (chimehoursstate == CHIMEHOURS) {
    chimehoursrt = ctime + CHIMETIME;
    bell1on();
    bell2on();
    bell3on();
    cqcount++;
    if (cqcount == cqhours) chimehoursen = false;
  }
}
    
void timeset() { // used to set time, task starts running when ffbutton or revbutton is pressed and stops running when steptime is back to normal clock rate
  timesetrt += TSCHECKTIME;
  msteptime = TSSTEPS[tsindex];
  if (ffbutton) { // accel
    if (normaldirection) { // going forward
      tsindex++; //increase speed for next time through
      if (tsindex > NUMTSSTEPS-1) tsindex = NUMTSSTEPS-1; //at top speed
    } else { // going backward
      tsindex--; //decrease speed for next time through
      if (tsindex < 0) {
        tsindex = 0;
        setnormaldirection(true);
      }
    }  
  } else if (revbutton) { // decel
    if (!normaldirection) { // going backward
      tsindex++; //increase speed for next time through
      if (tsindex > NUMTSSTEPS-1) tsindex = NUMTSSTEPS-1; //at top speed
    } else { // going forward
      tsindex--; //decrease speed for next time through
      if (tsindex < 0) {
        tsindex = 0;
        setnormaldirection(false);
      }
    }  
  } else { // no button pressed go toward 0
    tsindex--; // always slower forward or backward
    if (tsindex < 0) { // if back to slowest speed then set normal rate, going normal direction and disable timeset task    
      msteptime = MSTEPTIME;
      timeseten = false;
      tsindex = 0;
      setnormaldirection(true);
    }
  }
//Serial.print ("timeset: ctime="); Serial.print(ctime); Serial.print(" msteptime="); Serial.print(msteptime); Serial.print(" tsindex="); Serial.print(tsindex); Serial.print(" ffbutton="); Serial.println(ffbutton);
}


// Functions
void mstep() {
  digitalWrite(msteppin, HIGH);
  delayMicroseconds(10);              // wait for 10 us
  digitalWrite(msteppin, LOW);
  mstepcount++;
  if (normaldirection) { //forward direction - count up and when > 1, step hour
    hstepcountQ24 += H2MQ24; // increment Q24 version of counter
    if (hstepcountQ24 > N1PT0Q24) {
      hstep();
      hstepcountQ24 -= N1PT0Q24;
    }
  } else { //backward direction - count down and when less than 0, step hour
    if (hstepcountQ24 >= H2MQ24) {
      hstepcountQ24 -= H2MQ24;
    } else {
      hstep();
      hstepcountQ24 += (N1PT0Q24 - H2MQ24);
    }    
  }
}
  
void hstep() {
  digitalWrite(hsteppin, HIGH);
  delayMicroseconds(10);              // wait for 10 us
  digitalWrite(hsteppin, LOW);
  hstepcount++;
}

boolean timeup(unsigned long int tnow, unsigned long int tup) {
  /* 3 cases: both tnow and tup are on same side of overflow, easy case just return tnow>tup
  tnow has overflowed and tup hasn't, eg tnow=1, tup=2^32-2, rare case, but dangerous, need to return true or task may never run!
  tup has overflowed and tnow hasn't, eg tup=1, tnow=2^32-2, happenes every 70 min or so, not dangerous, but would cause task to run at wrong time
  using some long forgotten and now remembered properties of unsigned math here are examples of each case:
  
    tnow         tup      tnow-tup   timeup return
      100          50           50       true
       50         100      2^32-50      false
   2^32-2           2       2^32-4      false
        2      2^32-2            4       true
        
I'll compare tnow-tup to 2^31 if < then return true, if > return false, this allows max task time of ~35 min and task will still run if 35 min late...        
  */
  return ((tnow - tup) < HALFMAXUL); 
}

void setnormaldirection(boolean dir) {
    if (dir) {
      digitalWrite(clkdirpin, HIGH); //set forward direction
      normaldirection = true;
    } else {
      digitalWrite(clkdirpin, LOW); //set backwards direction
      normaldirection = false;
    } 
}


void checkbuttons()  { // sets globals for whether button is pressed ~120 us to run
// check one button per call to keep overhead down shouldn't be perceptible
  static int whichbutton = 0;
  if (whichbutton == 0) ffbutton = (analogRead(FFBTNPIN) < 500);  
  //ffbutton = true; //test for ff
  if (whichbutton == 1) revbutton = (analogRead(REVBTNPIN) < 500);
  if (whichbutton == 2) bell1button = (analogRead(B1BTNPIN) < 500);
  if (whichbutton == 3) bell2button = (analogRead(B2BTNPIN) < 500);
  if (whichbutton == 4) bell3button = (analogRead(B3BTNPIN) < 500);
  if (whichbutton == 5) bellswitch = (analogRead(BSWITCHPIN) < 500);
  whichbutton++;
  if (whichbutton > 5) whichbutton = 0;
  /*Serial.print(ffbutton);
  Serial.print(revbutton);
  Serial.print(bell1button);
  Serial.print(bell2button);
  Serial.print(bell3button);
  Serial.println(bellswitch);*/
}

void setup() {                
  // Initialize the bell pins to output and set drivers off
  pinMode(bell1pinA, OUTPUT);       
  pinMode(bell1pinB, OUTPUT);  
  digitalWrite(bell1pinA, LOW);
  digitalWrite(bell1pinB, LOW);
  pinMode(bell2pinA, OUTPUT);       
  pinMode(bell2pinB, OUTPUT);  
  digitalWrite(bell2pinA, LOW);
  digitalWrite(bell2pinB, LOW);
  pinMode(bell3pinA, OUTPUT);       
  pinMode(bell3pinB, OUTPUT);  
  digitalWrite(bell3pinA, LOW);
  digitalWrite(bell3pinB, LOW);

  // Initialize stepper motor pins to output, enable, set direction, and step
  pinMode(clkdirpin, OUTPUT);     
  pinMode(msteppin, OUTPUT);
  pinMode(hsteppin, OUTPUT);
  pinMode(clkenablepin, OUTPUT);
  digitalWrite(hsteppin, LOW);  // set step pins low
  digitalWrite(msteppin, LOW); 
  setnormaldirection(true); //set direction
  digitalWrite(clkenablepin, LOW); // enable stepper drives
  Serial.begin(9600);    
  
  msteptime = MSTEPTIME; // set step time to normal timekeeping 
  
  // Start tasks
  ctime = micros();
  keeptimeen = true; keeptimert = ctime + MSTEPTIME;
  
  ctimeprev = micros();

}


void loop() {
  unsigned int d;
  int ain;
  int i;
  
  ctime = micros(); // stamp current time once per loop
  checkbuttons(); // loads button variables with current state
  
  // ring bells if bell buttons are pressed and not setting time
  // if button was just pressed and bell not already running then start it
  if (bellswitch && !timeseten) {
    if (bell1button && !bell1buttonprev && !bell1offen) bell1on();  
    if (bell2button && !bell2buttonprev && !bell2offen) bell2on();  
    if (bell3button && !bell3buttonprev && !bell3offen) bell3on(); 
  } 

  if ((ffbutton || revbutton) && !(bell1button || bell2button || bell3button)) { // go into time set mode if just forward or reverse pressed
    if (!timeseten) { // enable and initialize task if not already enabled
     timeseten = true;
     timesetrt = ctime;
     tsindex = 0;
    }
    
  }
  
  // reset time to 12:00 if all buttons pressed
  if (bell1button && bell2button && bell3button && ffbutton && revbutton) {
    hours = 12;
    minutes = 00;
    minQ30=0;
    Serial.println("Reset time");
  }
  
  // run enbabled and expired tasks
  if (keeptimeen && timeup(ctime, keeptimert)) keeptime();
  if (bell1onen && timeup(ctime, bell1onrt)) bell1on();
  if (bell1offen && timeup(ctime, bell1offrt)) bell1off();
  if (bell2onen && timeup(ctime, bell2onrt)) bell2on();
  if (bell2offen && timeup(ctime, bell2offrt)) bell2off();
  if (bell3onen && timeup(ctime, bell3onrt)) bell3on();
  if (bell3offen && timeup(ctime, bell3offrt)) bell3off();
  if (chimehoursen && timeup(ctime, chimehoursrt)) chimehours();
  if (timeseten && timeup(ctime, timesetrt)) timeset();
 
     
  //save previous button state
  bell1buttonprev = bell1button;
  bell2buttonprev = bell2button;
  bell3buttonprev = bell3button;
  ffbuttonprev = ffbutton;
  revbuttonprev = revbutton; 
  
/*
  if ((ctime - ctimeprev) > 350) { // ASSERT - debug only
    Serial.println("Overflow");
    Serial.print ("ctime="); Serial.print(ctime); Serial.print(" msteptime="); Serial.print(msteptime); Serial.print(" tsindex="); Serial.print(tsindex); Serial.print(" ffbutton="); Serial.println(ffbutton);
    Serial.print ("ctimeprev="); Serial.println(ctimeprev); 
    Serial.print ("keeptimeen="); Serial.print(keeptimeen); Serial.print(" keeptimert="); Serial.println(keeptimert);
    Serial.print ("chimehoursen="); Serial.print(chimehoursen); Serial.print(" chimehoursrt="); Serial.println(chimehoursrt);
    while (true);
  }
*/
  ctimeprev = ctime;
}
  
  

