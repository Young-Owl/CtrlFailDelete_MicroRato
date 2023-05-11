/**  MicroRato 2023 - Ctrl+Fail+Delete
  *  2022/2023
  *  UA - Universidade de Aveiro
  *  DETI - Departamento de Electrónica, Telecomunicações e Informática
*/

/* Every generation = Add a new line: */
/*          CTRL+FAIL+DELETE          */ 

#include <Arduino.h>
#include <Encoder.h>

/* PWM Pins: 3, 5, 6, 9, 10, 11 */

/* Defining PINs for the L298N Module */
#define M1_EN     11  /**< Motor 1 Speed Control. (PWM)   */
#define M1_CLOCK  2   /**< Motor 1 Clockwise Control.     */
#define M1_ACLOCK 3   /**< Motor 1 Anti-Clockwise Control.*/
#define M2_EN     10  /**< Motor 2 Speed Control. (PWM)   */
#define M2_CLOCK  4   /**< Motor 2 Clockwise Control.     */
#define M2_ACLO   5   /**< Motor 2 Anti-Clockwise Control.*/

/* Defining PINs for the Encoder Module */
#define ENCODER1_A 7  /**< Motor 1 Encoder A.             */
#define ENCODER1_B 6    /**< Motor 1 Encoder B.             */
#define ENCODER2_A 9  /**< Motor 2 Encoder A.             */
#define ENCODER2_B 8  /**< Motor 2 Encoder B.             */

volatile int time = 0;
volatile int lastPulse = 0;
volatile int pinCurrentState = 0;
volatile int pinLastState = 0;
volatile bool direction = false;  /**< false = Anti-Clockwise, true = Clockwise */

void pulseDetect(){
  pinCurrentState = digitalRead(ENCODER1_A);

  if(pinCurrentState == HIGH && pinLastState == LOW){
    if(digitalRead(ENCODER1_B) == LOW && direction){
      direction = false;
    }
    else if(digitalRead(ENCODER1_B) == HIGH && !direction){
      direction = true;
    }
  }
  if(direction) { time++; }
  else          { time--; }

  pinLastState = pinCurrentState;
}

void setup()
{
  Serial.begin(9600);

  /* Define PinArray */
  int outPins[] = {M1_EN, M1_CLOCK, M1_ACLOCK, M2_EN, M2_CLOCK, M2_ACLO};
  int inPins[] = {ENCODER1_A, ENCODER1_B, ENCODER2_A, ENCODER2_B};

  /* Define PinModes */
  for(int i = 0; i < sizeof(outPins); i++){
    pinMode(outPins[i], OUTPUT);
  }

  for(int i = 0;i < sizeof(inPins); i++){
    pinMode(inPins[i], INPUT);
  }

  /* Initialize the Enconder Interrupts */
  attachInterrupt(digitalPinToInterrupt(ENCODER1_A), pulseDetect, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_A), pulseDetect, CHANGE);

  
}

void loop()
{
  Serial.println(time);
  delay(1000);
}

