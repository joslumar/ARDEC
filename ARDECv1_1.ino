/******************************************************************************************************************************
 *
 * Firmware para programar un ATMEGA328 con IDE Arduino
 * Sustituye funcionalidad del controlador de motores de dos ejes para EQ4 EQ5
 *
 * (C) Jose Luis Marquez Regueiro
 *
 * GPL Licensed (see http://www.fsf.org/copyleft/gpl.txt)
 *
 * Este programa se distribuye SIN GARANTIA de ningun tipo.
 *
 ******************************************************************************************************************************/


#include <TimerOne.h> // Ojo, que me cargo las salidas PWM 9 y 10
// ... y mas ojo, que si activo el serial.print altero el temporizador ... o quizs el tratamiento de interrupciones
// o si no, mira el control de los motores con el osciloscopio o el analizador lgoico
//#include <EEPROM.h>


#define B_AR_UP    0b0010
#define B_AR_DOWN  0b0001
#define B_DEC_UP   0b1000
#define B_DEC_DOWN 0b0100

#define B_AR       0b0011
#define B_DEC      0b1100

#define AR_MICROSTEPS 8 // 4 8
#define AR_ULTIMOPASO (AR_MICROSTEPS - 1)

#define DEC_MICROSTEPS 8 // 4 8
#define DEC_ULTIMOPASO (DEC_MICROSTEPS - 1)

#define MAXRELAX_DEC  250

#define PIN_INFO_LED    8
#define PIN_PWR_LED     7

#define PIN_BOBINA_AR_1A 11 // PWM
#define PIN_BOBINA_AR_1B 13 // PWM
#define PIN_BOBINA_AR_2A 5  // PWM
#define PIN_BOBINA_AR_2B 6  // PWM

#define PIN_BOBINA_DEC_1A  4   // no PWM
#define PIN_BOBINA_DEC_1B  10  // no PWM
#define PIN_BOBINA_DEC_2A  9   // no PWM
#define PIN_BOBINA_DEC_2B  12  // no PWM


#define PIN_AR_UP    3    // AR+
#define PIN_AR_DOWN A4    // AR-
#define PIN_DEC_UP  A5    // DEC+
#define PIN_DEC_DOWN 2    // DEC-

#define PIN_NS     A3
#define PIN_SPEED0 A2
#define PIN_SPEED1 A1

#define UP   0
#define DOWN 1
#define NORTH 0
#define SOUTH 1

/*
 Motor Step Angle: 7.5 degrees => 48 steps / turn
 Motor Gear Ratio: 120:1
 Worm Gear Ratio:  144:1
 
 1 step   = 1.5625  arcsec
 1/2 step = 0.78125 arcsec
 
 Secconds in a day:  24*60*60*0.99726958 = 86164s (23 hours, 56 minutes, 4.0916 seconds)
 
 4.9863s = 1 motor turns => 1s sidereal time => v motor = 9.6263 steps / s = 19.253 usteps / s [1/2 step]
 
 
 single step:
 sidereal time (t step) = 103.88 ms
 max speed = 8x => tick =  12.985 ms
 max speed = 16x => tick =  6.4926 ms // no lo aguanta el motor de la EQ5 (los motores del dual axis son lentos de c*jones)
 max speed = 32x => tick =  3.2462 ms // no lo aguanta el motor de la EQ5
 
 1/2 step:
 sidereal time (t step) = 51.941 ms
 max speed = 8x => tick =  6.4926 ms  // mitad de consumo que single step  <- tamaños de pulso en AR de 155,832ms(H) y 259,720ms(L)
 max speed = 16x => tick =  3.2462 ms // no lo aguanta el motor de la EQ5
 max speed = 32x => tick =  1.6231 ms // no lo aguanta el motor de la EQ5
 */

#define SPEED_MAX  8
#define TICK_USEC 6493+2 // half step 8x
////#define TICK_USEC 12985+0 // single step 8x

// deberia poder corregir el decimal del tamaño preciso de tick haciendo perder un tick de vez en cuando

volatile unsigned long ticks = 0;
volatile unsigned long dec_ticks = 0;
volatile unsigned long ar_ticks = 0;

volatile boolean interrupting_t1 = false;

volatile boolean hemisphere = NORTH; // NORTH SOUTH
volatile byte motion_speed  = 4;   // 2, 4, 8
volatile boolean superspeed = false;

volatile boolean priomotion_ar = false;   // motor AR a toda leche
volatile boolean priomotion_dec = false;  // motor DEC a toda leche

volatile short int paso_AR  = 0;  // 0 .. AR_ULTIMOPASO
volatile short int paso_DEC = 0;  // 0 .. DEC_ULTIMOPASO

volatile int delay_tick_ar;  // tiempo muerto mientras espera el siguiente slot tick para cumplir con la velocidad del mando
volatile int delay_tick_dec; // tiempo muerto mientras espera el siguiente slot tick para cumplir con la velocidad del mando

volatile int ar_motorPos   = 0;   // paso en curso del motor AR
volatile int dec_motorPos  = 0;   // paso en curso del motor DEC
volatile int ar_targetPos  = 0;   // paso deseado del motor AR
volatile int dec_targetPos = 0;   // paso deseado del motor DEC

volatile int relax_dec = 0;          // solo relajo el DEC

int pulsado;
int boton;

//---------------------------------

void relaja_motor_AR() {
  digitalWrite(PIN_BOBINA_AR_1A, LOW);
  digitalWrite(PIN_BOBINA_AR_1B, LOW);  
  digitalWrite(PIN_BOBINA_AR_2A, LOW);
  digitalWrite(PIN_BOBINA_AR_2B, LOW);
}

void relaja_motor_DEC() {
  digitalWrite(PIN_BOBINA_DEC_1A, LOW);
  digitalWrite(PIN_BOBINA_DEC_1B, LOW);  
  digitalWrite(PIN_BOBINA_DEC_2A, LOW);
  digitalWrite(PIN_BOBINA_DEC_2B, LOW);
}

void relaja_motores() {
  relaja_motor_AR();
  relaja_motor_DEC();
}

//---------------------------------------------------------------------------
//---------- AR MOTOR

#if (AR_MICROSTEPS == 4)
void excita_motor_AR(int paso) {
  switch(paso) {
  case 0:         //paso 0
    digitalWrite(PIN_BOBINA_AR_1A, LOW);
    digitalWrite(PIN_BOBINA_AR_1B, HIGH);  
    digitalWrite(PIN_BOBINA_AR_2A, HIGH);
    digitalWrite(PIN_BOBINA_AR_2B, LOW);
    break;
  case 1:         //paso 1
    digitalWrite(PIN_BOBINA_AR_1A, LOW);
    digitalWrite(PIN_BOBINA_AR_1B, HIGH);  
    digitalWrite(PIN_BOBINA_AR_2A, LOW);
    digitalWrite(PIN_BOBINA_AR_2B, HIGH);
    break;
  case 2:         //paso 2
    digitalWrite(PIN_BOBINA_AR_1A, HIGH);
    digitalWrite(PIN_BOBINA_AR_1B, LOW);  
    digitalWrite(PIN_BOBINA_AR_2A, LOW);
    digitalWrite(PIN_BOBINA_AR_2B, HIGH);
    break;
  case 3:         //paso 3
    digitalWrite(PIN_BOBINA_AR_1A, HIGH);
    digitalWrite(PIN_BOBINA_AR_1B, LOW);  
    digitalWrite(PIN_BOBINA_AR_2A, HIGH);
    digitalWrite(PIN_BOBINA_AR_2B, LOW);
    break;
  }
}

#elif (AR_MICROSTEPS == 8)
// medios pasos
void excita_motor_AR(int paso) {
  switch(paso) {
  case 0:         //paso 0
    digitalWrite(PIN_BOBINA_AR_1A, LOW);
    digitalWrite(PIN_BOBINA_AR_1B, HIGH);
    digitalWrite(PIN_BOBINA_AR_2A, HIGH);
    digitalWrite(PIN_BOBINA_AR_2B, LOW);
    break;
  case 1:         //paso 1
    digitalWrite(PIN_BOBINA_AR_1A, LOW);
    digitalWrite(PIN_BOBINA_AR_1B, HIGH);  
    digitalWrite(PIN_BOBINA_AR_2A, LOW);
    digitalWrite(PIN_BOBINA_AR_2B, LOW);
    break;
  case 2:         //paso 2
    digitalWrite(PIN_BOBINA_AR_1A, LOW);
    digitalWrite(PIN_BOBINA_AR_1B, HIGH);  
    digitalWrite(PIN_BOBINA_AR_2A, LOW);
    digitalWrite(PIN_BOBINA_AR_2B, HIGH);
    break;
  case 3:         //paso 3
    digitalWrite(PIN_BOBINA_AR_1A, LOW);
    digitalWrite(PIN_BOBINA_AR_1B, LOW);  
    digitalWrite(PIN_BOBINA_AR_2A, LOW);
    digitalWrite(PIN_BOBINA_AR_2B, HIGH);
    break;
  case 4:         //paso 4
    digitalWrite(PIN_BOBINA_AR_1A, HIGH);
    digitalWrite(PIN_BOBINA_AR_1B, LOW);  
    digitalWrite(PIN_BOBINA_AR_2A, LOW);
    digitalWrite(PIN_BOBINA_AR_2B, HIGH);
    break;
  case 5:         //paso 5
    digitalWrite(PIN_BOBINA_AR_1A, HIGH);
    digitalWrite(PIN_BOBINA_AR_1B, LOW);  
    digitalWrite(PIN_BOBINA_AR_2A, LOW);
    digitalWrite(PIN_BOBINA_AR_2B, LOW);
    break;
  case 6:         //paso 6
    digitalWrite(PIN_BOBINA_AR_1A, HIGH);
    digitalWrite(PIN_BOBINA_AR_1B, LOW);  
    digitalWrite(PIN_BOBINA_AR_2A, HIGH);
    digitalWrite(PIN_BOBINA_AR_2B, LOW);
    break;
  case 7:         //paso 7
    digitalWrite(PIN_BOBINA_AR_1A, LOW);
    digitalWrite(PIN_BOBINA_AR_1B, LOW);  
    digitalWrite(PIN_BOBINA_AR_2A, HIGH);
    digitalWrite(PIN_BOBINA_AR_2B, LOW);
    break;
  }
}
#endif

//---------------------------------------------------------------------------
//---------- DEC MOTOR

#if (DEC_MICROSTEPS == 4)
void excita_motor_DEC(int paso) {
  switch(paso) {
  case 0:         //paso 0
    digitalWrite(PIN_BOBINA_DEC_1A, LOW);
    digitalWrite(PIN_BOBINA_DEC_1B, HIGH);  
    digitalWrite(PIN_BOBINA_DEC_2A, HIGH);
    digitalWrite(PIN_BOBINA_DEC_2B, LOW);
    break;
  case 1:         //paso 1
    digitalWrite(PIN_BOBINA_DEC_1A, LOW);
    digitalWrite(PIN_BOBINA_DEC_1B, HIGH);  
    digitalWrite(PIN_BOBINA_DEC_2A, LOW);
    digitalWrite(PIN_BOBINA_DEC_2B, HIGH);
    break;
  case 2:         //paso 2
    digitalWrite(PIN_BOBINA_DEC_1A, HIGH);
    digitalWrite(PIN_BOBINA_DEC_1B, LOW);  
    digitalWrite(PIN_BOBINA_DEC_2A, LOW);
    digitalWrite(PIN_BOBINA_DEC_2B, HIGH);
    break;
  case 3:         //paso 3
    digitalWrite(PIN_BOBINA_DEC_1A, HIGH);
    digitalWrite(PIN_BOBINA_DEC_1B, LOW);  
    digitalWrite(PIN_BOBINA_DEC_2A, HIGH);
    digitalWrite(PIN_BOBINA_DEC_2B, LOW);
    break;
  }
}

#elif (DEC_MICROSTEPS == 8)
// medios pasos
void excita_motor_DEC(int paso) {
  switch(paso) {
  case 0:         //paso 0
    digitalWrite(PIN_BOBINA_DEC_1A, LOW);
    digitalWrite(PIN_BOBINA_DEC_1B, HIGH);
    digitalWrite(PIN_BOBINA_DEC_2A, HIGH);
    digitalWrite(PIN_BOBINA_DEC_2B, LOW);
    break;
  case 1:         //paso 1
    digitalWrite(PIN_BOBINA_DEC_1A, LOW);
    digitalWrite(PIN_BOBINA_DEC_1B, HIGH);  
    digitalWrite(PIN_BOBINA_DEC_2A, LOW);
    digitalWrite(PIN_BOBINA_DEC_2B, LOW);
    break;
  case 2:         //paso 2
    digitalWrite(PIN_BOBINA_DEC_1A, LOW);
    digitalWrite(PIN_BOBINA_DEC_1B, HIGH);  
    digitalWrite(PIN_BOBINA_DEC_2A, LOW);
    digitalWrite(PIN_BOBINA_DEC_2B, HIGH);
    break;
  case 3:         //paso 3
    digitalWrite(PIN_BOBINA_DEC_1A, LOW);
    digitalWrite(PIN_BOBINA_DEC_1B, LOW);  
    digitalWrite(PIN_BOBINA_DEC_2A, LOW);
    digitalWrite(PIN_BOBINA_DEC_2B, HIGH);
    break;
  case 4:         //paso 4
    digitalWrite(PIN_BOBINA_DEC_1A, HIGH);
    digitalWrite(PIN_BOBINA_DEC_1B, LOW);  
    digitalWrite(PIN_BOBINA_DEC_2A, LOW);
    digitalWrite(PIN_BOBINA_DEC_2B, HIGH);
    break;
  case 5:         //paso 5
    digitalWrite(PIN_BOBINA_DEC_1A, HIGH);
    digitalWrite(PIN_BOBINA_DEC_1B, LOW);  
    digitalWrite(PIN_BOBINA_DEC_2A, LOW);
    digitalWrite(PIN_BOBINA_DEC_2B, LOW);
    break;
  case 6:         //paso 6
    digitalWrite(PIN_BOBINA_DEC_1A, HIGH);
    digitalWrite(PIN_BOBINA_DEC_1B, LOW);  
    digitalWrite(PIN_BOBINA_DEC_2A, HIGH);
    digitalWrite(PIN_BOBINA_DEC_2B, LOW);
    break;
  case 7:         //paso 7
    digitalWrite(PIN_BOBINA_DEC_1A, LOW);
    digitalWrite(PIN_BOBINA_DEC_1B, LOW);  
    digitalWrite(PIN_BOBINA_DEC_2A, HIGH);
    digitalWrite(PIN_BOBINA_DEC_2B, LOW);
    break;
  }
}
#endif

//-----------------------------------------------------------------------

int siguiente_paso_AR(int sentido) {
  int rc=0;

  switch (hemisphere) {
  case NORTH:
    if ( sentido ) {
      if ( paso_AR < AR_ULTIMOPASO )
        paso_AR++;
      else
        paso_AR = 0;

      ar_motorPos++;
    }
    else {
      if ( paso_AR > 0 )
        paso_AR--;
      else
        paso_AR = AR_ULTIMOPASO;

      ar_motorPos--;
    }
    break;
  case SOUTH:
    if ( sentido ) {
      if ( paso_AR > 0 )
        paso_AR--;
      else
        paso_AR = AR_ULTIMOPASO;

      ar_motorPos++;
    }
    else {
      if ( paso_AR < AR_ULTIMOPASO )
        paso_AR++;
      else
        paso_AR = 0;

      ar_motorPos--;
    }
    break;
  }
  rc = 1;
  return(rc);
}

int siguiente_paso_DEC(int sentido) {
  int rc=0;

  if ( sentido ) {
    if ( paso_DEC < DEC_ULTIMOPASO )
      paso_DEC++;
    else
      paso_DEC = 0;

    dec_motorPos++;
  }
  else {
    if ( paso_DEC > 0 )
      paso_DEC--;
    else
      paso_DEC = DEC_ULTIMOPASO;

    dec_motorPos--;
  }
  rc = 1;
  return(rc);
}


///-----------------------

void interrup_t1() {
  int diff_ar = 0;
  int diff_dec = 0;

  if (!interrupting_t1) {
    interrupting_t1 = true;

    if ( ticks % SPEED_MAX == 0) {   // sidereal (vel max 8x), un slot de 8
      ar_targetPos++;
      //Serial.print("T:"); 
      //Serial.print(ticks); 
      //Serial.print("\n");
      delay_tick_ar = 0; // ya!
    }
    //---------------- MOTOR AR
    //
    if ( delay_tick_ar == 0 || priomotion_ar ) { // no hay que esperar mas
      delay_tick_ar =  SPEED_MAX/motion_speed - 1;  // velocidad 8x => no espera (11111111) ; velocidad 2x => espera 3 (10001000)
      diff_ar = ar_targetPos - ar_motorPos;
      if ( diff_ar ) { // si hay pasos pendientes, mueve el motor
        if ( siguiente_paso_AR( diff_ar >0 ) == 1 )  { // calcula siguiente paso AR
          excita_motor_AR(paso_AR); // mueve al siguiente paso AR
        }
        diff_ar = ar_targetPos - ar_motorPos; // vuelve a calcular pasos pendientes
        /*
         Serial.print("AR:"); 
         Serial.print(ticks);
         Serial.print(" "); 
         Serial.print(diff_ar); 
         Serial.print("\n");
         */
      }
    }
    else { // hay que esperar
      if ( delay_tick_ar ) {
        delay_tick_ar--;
      }
    }
    //---------------- MOTOR DEC
    //
    if ( delay_tick_dec == 0 || priomotion_dec ) {  // tick es cero o movimiento prioritario
      delay_tick_dec =  SPEED_MAX/motion_speed - 1;  // velocidad 8x => no espera (11111111) ; velocidad 2x => espera 3 (10001000)
      diff_dec = dec_targetPos - dec_motorPos; // calcula ticks pendientes
      if ( diff_dec ) {  // si hay pasos pendientes, mueve el motor
        if ( siguiente_paso_DEC( diff_dec > 0 ) == 1 ) { // calcula siguiente paso de motor DEC
          excita_motor_DEC(paso_DEC); // mueve al siguiente paso de motor DEC
        }
        diff_dec = dec_targetPos - dec_motorPos;  // vuelve a calcular pasos pendientes
        /*
        Serial.print("DEC:"); 
         Serial.print(ticks);
         Serial.print(" "); 
         Serial.print(diff_dec); 
         Serial.print(delay_tick_dec); 
         Serial.print("\n");
         */
        relax_dec = MAXRELAX_DEC;  // se ha movido, entonces no relajes por MAXRELAX_DEC
      }
    } 
    else { // hay que esperar ranura
      if ( delay_tick_dec > 0 ) {
        delay_tick_dec--;
      }
    }
    if ( relax_dec > 0 ) { // si estoy esperando relajacion, comienza la cuenta atras ...
      relax_dec--;
      if ( relax_dec == 0 ) { 
        //Serial.print("RELAJANDO\n"); 
        relaja_motor_DEC();
      }
    }
    else
      relax_dec = MAXRELAX_DEC; // espera un tiempo (ticks) hasta relajacion de motor DEC (el AR nunca se relaja)
  }

  if( millis()/1000 % 2) { // cada segundo, keepalive
    digitalWrite(PIN_PWR_LED, HIGH);   // turn the LED on
  }
  else
    digitalWrite(PIN_PWR_LED, LOW);    // turn the LED off

  // calibracion
  // stty -F /dev/ttyUSB0 raw ;  ts "%.s" < /dev/ttyUSB0  | \
  // awk 'BEGIN {t=0} ; {if (t==0) {s=$1;t=$3} else {print "s:"$1" t:"$3" T:"($1-s)/($3-t)*1000}}'

  if( ticks % 1000 == 0) { // cada 1000 ticks 
    Serial.print(millis());
    Serial.print(" ");
    Serial.print(ticks);
    Serial.print("\n"); 
  }

  ticks++;
  interrupting_t1 = false; // fin interrupcion
}

void stopAR() {  
  delay_tick_ar = 0;
  ar_targetPos = ar_motorPos;
}

void stopDEC() {  
  delay_tick_dec = 0;
  dec_targetPos = dec_motorPos;
}

void moveAR(byte sentido,int steps,boolean prio) {
  switch( sentido ) {
  case UP:
    ar_targetPos += steps;
    break;
  case DOWN:
    ar_targetPos -= steps;
    break;
  }
  if (prio)
    delay_tick_ar = 0;

}

void moveDEC(byte sentido,int steps,boolean prio) {
  switch( sentido ) {
  case UP:
    dec_targetPos += steps;
    break;
  case DOWN:
    dec_targetPos -= steps;
    break;
  }
  if (prio)
    delay_tick_dec = 0;
}

//--------------------------------------------------------------------

int botones() {
  int rc = 0;

  if( digitalRead(PIN_AR_UP)== LOW)    rc += B_AR_UP;
  if( digitalRead(PIN_AR_DOWN)== LOW)  rc += B_AR_DOWN;
  if( digitalRead(PIN_DEC_UP)== LOW)   rc += B_DEC_UP;
  if( digitalRead(PIN_DEC_DOWN)== LOW) rc += B_DEC_DOWN;

  return(rc);
}


byte speed_switches() {
  byte s = (digitalRead(PIN_SPEED1) << 1 ) + digitalRead(PIN_SPEED0);
  switch (s) {
  case 0:
    return(2);  // 2x
    break;
  case 1:
    return(4);  // 4x
    break;
  case 2:
    return(8);  // 8x
    break;
  }
}

volatile byte sentido_ar, sentido_dec;
void botonera() {
  boolean ar_prio = false, dec_prio = false;

  if( millis()/1000 % 2 ) {// cada segundo leo la velocidad del switch
    if (!superspeed)
      motion_speed = speed_switches();
    else
      motion_speed = SPEED_MAX;
  }
  boton = botones();
  switch( boton ) {
  case B_AR_DOWN:
    sentido_ar = DOWN;
    break;
  case B_AR_UP:
    sentido_ar = UP;
    break;
  case B_DEC_DOWN:
    sentido_dec = DOWN;
    break;
  case B_DEC_UP:
    sentido_dec = UP;
    break;
  }

  if (( boton & B_AR)) {
    if ( !pulsado ) {     // no estaba pulsado antes
      ar_prio = true;
    }
    moveAR(sentido_ar,1,ar_prio);
    pulsado = true;
    digitalWrite(PIN_INFO_LED, HIGH);   // turn the LED on
    /*
    Serial.print("AR:"); 
     Serial.print(ar_motorPos);
     Serial.print("\t");
     Serial.print(motion_speed);
     Serial.print("\n");
     */
    if ( boton == B_AR )
      superspeed=true;
    else  
      superspeed=false;

  } 
  else if ( boton & B_DEC ) {
    if ( !pulsado ) {      // no estaba pulsado antes
      dec_prio = true;
    }
    moveDEC(sentido_dec,1,dec_prio);
    pulsado = true;
    digitalWrite(PIN_INFO_LED, HIGH);   // turn the LED on

    /*
    Serial.print("DEC:"); 
     Serial.print(dec_motorPos);
     Serial.print("\t");
     Serial.print(motion_speed);
     Serial.print("\n");
     */

    if ( boton == B_DEC )
      superspeed=true;
    else  
      superspeed=false;
  } 
  else { // no se han pulsado botones ahora
    if ( pulsado ) {   // estaba pulsado antes, pero ahora no. Entonces paro de actuar motores
      stopDEC();
      stopAR();
    } 
    else
      digitalWrite(PIN_INFO_LED, LOW);   // turn the LED off

    pulsado = false;
    superspeed=false;
  }
}

void setup() {

  // initialize the buttons pin as a input:
  pinMode(PIN_DEC_UP, INPUT);
  digitalWrite(PIN_DEC_UP, HIGH);       // turn on pullup resistors
  pinMode(PIN_AR_UP, INPUT);
  digitalWrite(PIN_AR_UP, HIGH);
  pinMode(PIN_AR_DOWN, INPUT);
  digitalWrite(PIN_AR_DOWN, HIGH); 
  pinMode(PIN_DEC_DOWN, INPUT);
  digitalWrite(PIN_DEC_DOWN, HIGH);
  pinMode(PIN_NS, INPUT);
  digitalWrite(PIN_NS, HIGH);
  pinMode(PIN_SPEED0, INPUT);
  digitalWrite(PIN_SPEED0, HIGH);
  pinMode(PIN_SPEED1, INPUT);
  digitalWrite(PIN_SPEED1, HIGH);

  // initialize the LEDs as an output
  pinMode(PIN_PWR_LED, OUTPUT);  
  digitalWrite(PIN_PWR_LED, LOW);   // turn the LED off
  pinMode(PIN_INFO_LED, OUTPUT);  
  digitalWrite(PIN_INFO_LED, LOW);   // turn the LED off

  // initialize the AR motor phases
  pinMode(PIN_BOBINA_AR_1A, OUTPUT);
  pinMode(PIN_BOBINA_AR_1B, OUTPUT);
  pinMode(PIN_BOBINA_AR_2A, OUTPUT);
  pinMode(PIN_BOBINA_AR_2B, OUTPUT);

  // initialize the DEC motor phases
  pinMode(PIN_BOBINA_DEC_1A, OUTPUT);
  pinMode(PIN_BOBINA_DEC_1B, OUTPUT);
  pinMode(PIN_BOBINA_DEC_2A, OUTPUT);
  pinMode(PIN_BOBINA_DEC_2B, OUTPUT);

  if(digitalRead(PIN_NS)== LOW)
    hemisphere = NORTH;
  else
    hemisphere = SOUTH;

  excita_motor_AR(paso_AR);
  excita_motor_DEC(paso_DEC);

  // initialize timer1
  Timer1.initialize(TICK_USEC); // Note that this breaks analogWrite() for digital pins 9 and 10 on Arduino.
  Timer1.attachInterrupt(interrup_t1);

  // initialize serial communication:
  Serial.begin(9600L);
  digitalWrite(PIN_INFO_LED, HIGH);   // turn the LED off
  delay(500);  
  digitalWrite(PIN_INFO_LED, LOW);   // turn the LED off
}


void loop() {
  botonera();
  delay(5);
}

// Local Variables:
// mode: c++
// End:

