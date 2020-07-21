/*

27-04-2020
- Improved S-Curve by Hashir.
- Changed hardware step pin to 9 from 11
- Changed microstepping from 2 to 4
- Tested max speed
--1:3/30/800 - 840RPM - 11.2kHz - #RUN 5600 89 1
- Tested min speed
--1:1/8/200 - 28RPM - 0.4kHz - #RUN 1400 2679 1
- Changed home fixed distance to approx 500ml.

pstm
*/
#include <Arduino.h>
#include "SerialCommands.h"
#include "header.h"

/**************************Private Defined*****************************/
#define HOME_DIR false
#define acce 0.33
#define decce 0.33

/**************************Private Veriables*****************************/
bool atHome = false;
int STATE = STALL;
bool LS1_STATUS = false, LS2_STATUS = false;

float t = 0.0;
float Tt;
float ta = 0.0;
float td = 0.0;
float minSpeed = 400.0;
float maxSpd = 0.0;
float mySpeed = 0.0;

unsigned long cM = 0; //current micros
unsigned long pM = 0; //previous micros
unsigned long dT = 0; //delta micros
unsigned v = 0;       //Time period for pulse out

// variables shared between interrupt context and main program
// context must be declared "volatile".
volatile unsigned long PULSECOUNT = 0;
unsigned long PULSETARGET = 0;

char serial_command_buffer_[32];
/***************************Private Functions Prototype**************/
void TIM2_IRQ(void);
void CMD_RUN(SerialCommands *sender);
void CMD_STOP(SerialCommands *sender);
void CMD_HOME(SerialCommands *sender);
void disablePulseOut(void);

/***************************Constructors****************************/
SerialCommand cmd_run_("#RUN", CMD_RUN);
SerialCommand cmd_stop_("#STOP", CMD_STOP);
SerialCommand cmd_home_("#HOME", CMD_HOME);
SerialCommands serial_commands_(&Serial, serial_command_buffer_, sizeof(serial_command_buffer_), "\r", " ");

HardwareTimer *Timer2 = new HardwareTimer(TIM2); // Timer 2 for PWM on PA_0

//This is the default handler, and gets called when no other command matches.
void cmd_unrecognized(SerialCommands *sender, const char *cmd)
{
  sender->GetSerial()->print("CMD ERROR [");
  sender->GetSerial()->print(cmd);
  sender->GetSerial()->println("]");
}

/**************************Interrupt Fucntions*****************************/
#ifdef __AVR__
ISR(TIMER1_COMPA_vect)
{
  if (STATE == STALL)
  {
    disablePulseOut();
    return;
  }

  PULSECOUNT++;
  ICR1 = v;
  OCR1A = ICR1 * 0.5f; //Set time period and 50% Duty Cycle
}

#elif STM32F1xx
/**
  * @brief This function handles TIM1 capture compare interrupt.
  */
void TIM2_IRQ(void)
{
  if (STATE == STALL)
  {
    disablePulseOut();
    return;
  }
  PULSECOUNT++;

  Timer2->setOverflow(v, TICK_FORMAT);                      //Set ARR register of the system for frequency of the system form 1Mhz(1) to 15Hz (65534)
  Timer2->setCaptureCompare(1, v / 2, TICK_COMPARE_FORMAT); // set capture compare register for duty cycle (ARR/x%)
}
#endif //End interrupt functions

/**
  * @brief  Disable Timer Interrupt as well as PWM
  * @param  void
  * @retval void
  */
void disablePulseOut(void)
{
#ifdef __AVR__
  TCCR1A &= ~(1 << COM1A1); //Disable pulse output through Timer 1
  TIMSK1 &= ~(1 << OCIE1A); //Disable Interrupt of Timer 1
#elif defined(STM32F1xx)
  Timer2->setCaptureCompare(1, 0, TICK_COMPARE_FORMAT); // set capture compare register for duty cycle (ARR/x%)
  Timer2->pause();
#endif
}

/**
  * @brief  Initialize all IOs, Timers for PWM and Serial Uarts
  * @param  non
  * @retval non
  */
void setup()
{
  /**************************Pin Initialization*****************************/
  pinMode(PWM_OUT, OUTPUT); //Pulse Output
  pinMode(DIR_PIN, OUTPUT);
  pinMode(LS1_PIN, INPUT);
  pinMode(LS2_PIN, INPUT);
  pinMode(EN1_PIN, OUTPUT);
  pinMode(EN2_PIN, OUTPUT);

  /**************************Serial Initialzation **************************/
  Serial.begin(115200);

  serial_commands_.SetDefaultHandler(cmd_unrecognized);
  serial_commands_.AddCommand(&cmd_run_);
  serial_commands_.AddCommand(&cmd_stop_);
  serial_commands_.AddCommand(&cmd_home_);

  /**************************Timer Initialzation **************************/
#ifdef __AVR__
  cli();
  DDRB |= (1 << PB1); // Set PB1 to be an output (Pin9 Arduino UNO)
  TCCR1A = 0;
  TCCR1B = 0; // Clear Timer/Counter Control Registers
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM12) | (1 << WGM13); // Set fast PWM Mode 14
  TCCR1B |= (1 << CS11);                 //TCCR1B |= (1 << CS10); // Set prescaler to 8 and starts PWM
  ICR1 = 65534;
  OCR1A = ICR1 / 2; // Set PWM frequency/top value and 50% Duty Cycle
  sei();
#elif STM32F1xx
  //Timer 2
  Timer2->pause();                                     //stop Timer 2
  Timer2->setMode(1, TIMER_OUTPUT_COMPARE_PWM1, PA_0); // set PWM mode with PWM pin
  Timer2->setOverflow(65534, TICK_FORMAT);             // set ARR of the TIMER2 to 65334
  Timer2->setPrescaleFactor(168);                      // set prescalar 168Mhz/168 = 1MHz
  Timer2->attachInterrupt(TIM2_IRQ);                   // set call back function
  Timer2->refresh();                                   // update all the parameters
  Timer2->pause();                                     // dont start timer yet
#endif
}

/**
  * @brief  Unparse data of serial commands
  * @param  Serial structure
  * @retval void
  */
void CMD_RUN(SerialCommands *sender)
{
  if (STATE == RUNNING_FWD || STATE == RUNNING_REV || STATE == HOMING)
    return;

  // Read Count
  char *count_str = sender->Next();
  if (count_str == NULL)
  {
    sender->GetSerial()->println("ERROR NO COUNT");
    return;
  }

  // Read Period
  char *period_str = sender->Next();
  if (period_str == NULL)
  {
    sender->GetSerial()->println("ERROR NO PERIOD");
    return;
  }

  // Read Direction
  char *dir_str = sender->Next();
  if (dir_str == NULL)
  {
    sender->GetSerial()->println("ERROR NO DIRECTION");
    return;
  }

  int dir = atoi(dir_str);

  digitalWrite(DIR_PIN, dir);

  PULSETARGET = atol(count_str);
  long period = atol(period_str);

  Serial.println("#RUN 1");

  digitalWrite(EN1_PIN, true);
  digitalWrite(EN2_PIN, true);

  Tt = (float)(period * PULSETARGET) / 1e6;

  ta = Tt * acce;
  td = Tt * decce;

  maxSpd = PULSETARGET - (Tt * minSpeed);
  maxSpd = maxSpd / ((0.5 * (ta + td)) + (Tt - (ta + td)));
  maxSpd = maxSpd + minSpeed;

  if (dir == 1)
    STATE = RUNNING_FWD;
  else if (dir == 0)
    STATE = RUNNING_REV;

#ifdef __AVR__
  TCCR1A |= (1 << COM1A1); //Enable pulse output through Timer 1
  TIMSK1 |= (1 << OCIE1A); //Enable Interrupt of Timer 1
#elif defined(STM32F1xx)
  Timer2->resume();
#endif
  PULSECOUNT = 0; //Initialize pulse counter
  t = 0;          //Initialize time for S-Curve

  Serial.print("Tt: ");
  Serial.println(Tt, 2);
  Serial.print("St: ");
  Serial.println(STATE);
  Serial.print("mS: ");
  Serial.println(maxSpd, 2);
}

/**
  * @brief  Disable Enable pins of the Motor driver
  * @param  Serial Structure
  * @retval void
  */
void CMD_STOP(SerialCommands *sender)
{
  if (STATE != STALL)
  {
    Serial.println("#STOP 1");

    STATE = STALL;
    digitalWrite(EN1_PIN, false);
    digitalWrite(EN2_PIN, false);

    Serial.println("#STOP 2");
  }
}

/**
  * @brief  Set motor to homming condition
  * @param  Serial Structure
  * @retval void
  */
void CMD_HOME(SerialCommands *sender)
{

  if (STATE != STALL)
    return;

  digitalWrite(EN1_PIN, false);
  digitalWrite(EN2_PIN, false);

  // Read Period
  char *period_str = sender->Next();
  if (period_str == NULL)
  {
    sender->GetSerial()->println("ERROR NO PERIOD");
    return;
  }

  // Cmd Rcvd
  Serial.println("#HOME 1");

  digitalWrite(DIR_PIN, HOME_DIR);

  long period = atol(period_str);

  STATE = HOMING;

  PULSETARGET = 6400; //3500;//6400;

  Tt = (float)(period * PULSETARGET) / 1e6;

  ta = Tt * acce;
  td = Tt * decce;

  maxSpd = PULSETARGET - (Tt * minSpeed);
  maxSpd = maxSpd / ((0.5 * (ta + td)) + (Tt - (ta + td)));
  maxSpd = maxSpd + minSpeed;

  digitalWrite(EN1_PIN, true);
  digitalWrite(EN2_PIN, true);

#ifdef __AVR__
  TCCR1A |= (1 << COM1A1); //Enable pulse output through Timer 1
  TIMSK1 |= (1 << OCIE1A); //Enable Interrupt of Timer 1
#elif defined(STM32F1xx)
  Timer2->resume();
#endif
  PULSECOUNT = 0; //Initialize pulse counter
  t = 0;          //Initialize time for S-Curve
}

/**
  * @brief  set PWM values 
  * @param  void
  * @retval void
  */
void ProcessSCurve()
{
  if (t < ta)
  {
    mySpeed = (float)0.5 * (maxSpd - minSpeed) * (1 + sin((t / ta - 0.5) * PI));
    mySpeed = mySpeed + minSpeed;
    v = (long)(2e6 / (mySpeed)) - 1;
    if (v > 65534)
      v = 65535;
  }

  if (t >= (Tt - td))
  {
    if (t > Tt)
      t = Tt;
    mySpeed = (float)0.5 * (maxSpd - minSpeed) * (1 + sin(((t - Tt - td) / td + 0.5) * PI));
    mySpeed = mySpeed + minSpeed;
    if (mySpeed < minSpeed)
      mySpeed = minSpeed;
    v = (long)(2e6 / (mySpeed)) - 1;
    if (v > 65534)
      v = 65535;
  }
}

/**
  * @brief  main loop function
  * @param  void
  * @retval void
  */
void loop()
{

  //STATE-LESS
  serial_commands_.ReadSerial();
  LS1_STATUS = digitalRead(LS1_PIN);
  LS2_STATUS = !digitalRead(LS2_PIN);

  //Serial.println(LS1_STATUS);

  // RUN FWD
  if (STATE == RUNNING_FWD)
  {
    if (PULSECOUNT >= PULSETARGET)
    {
      disablePulseOut();
      PULSECOUNT = 0;
      STATE = STALL;
      digitalWrite(EN1_PIN, false);
      digitalWrite(EN2_PIN, false);
      Serial.println("#RUN 2");
    }
  }

  // RUN REV - FIND LS
  if (STATE == RUNNING_REV || STATE == HOMING)
  {

    if (LS1_STATUS)
      digitalWrite(EN1_PIN, false);
    if (LS2_STATUS)
      digitalWrite(EN2_PIN, false);

    if (LS1_STATUS && LS2_STATUS)
    {
      if (STATE == RUNNING_REV)
        Serial.println("#RUN 2");
      else if (STATE == HOMING)
        Serial.println("#HOME 2");

      STATE = STALL;
    }
  }

  cM = micros();
  dT = cM - pM;
  if (dT >= 1000)
  {
    if (STATE == RUNNING_FWD || STATE == RUNNING_REV || STATE == HOMING)
    {
      t = (float)(t + (dT * 1e-6));
      ProcessSCurve();
    }
    pM = cM;
  }
}
