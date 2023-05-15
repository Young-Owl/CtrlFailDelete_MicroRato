/**  MicroRato 2023 - Ctrl+Fail+Delete
  *  2022/2023
  *  UA - Universidade de Aveiro
  *  DETI - Departamento de Electrónica, Telecomunicações e Informática
  *  Gonçalo Rodrigues; Marco; João Tavares;
*/

/* Every generation = Add a new line: */
/*          CTRL+FAIL+DELETE          */ // Initial Code for Tests
/*          CTRL+FAIL+DELETE          */ // Iteration 0 - Basic sensor testing
/*          CTRL+FAIL+DELETE          */ // Iteration 1 - Sensors not working; Added 2 sensors + Color Sensor

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_tockn.h>

/* PINs SCL and SDA (I2C Protocol) will be used for the GY-521 Module (Gyroscope)*/

/* Defining PINs for the L298N Module */
#define M1_EN     2   /**< Motor 1 Speed Control. (PWM)   */
#define M1_CLOCK  53  /**< Motor 1 Clockwise Control.     */
#define M1_ACLOCK 3   /**< Motor 1 Anti-Clockwise Control.*/
#define M2_EN     3   /**< Motor 2 Speed Control. (PWM)   */
#define M2_CLOCK  51  /**< Motor 2 Clockwise Control.     */
#define M2_ACLOCK 5   /**< Motor 2 Anti-Clockwise Control.*/

/* Defining PINs for the 2 side IR sensors */
#define IRL_EN 4      /**< Right IR Sensor Led Enable */
#define IRL A0        /**< Right IR Sensor Analog Pin */
#define IRR_EN 5      /**< Left IR Sensor Led Enable  */
#define IRR A1        /**< Left IR Sensor Analog Pin  */

/* Defining PINs for the front color sensor */
#define S0 6          /**< Color Sensor S0 Pin */
#define S1 7          /**< Color Sensor S1 Pin */
#define S2 8          /**< Color Sensor S2 Pin */
#define S3 9          /**< Color Sensor S3 Pin */
#define OUT 10        /**< Color Sensor OUT Pin */

/* Defining PINs for the START and STOP Buttons */
#define START 11      /**< Start Button Pin */
#define STOP 12       /**< Stop Button Pin  */

/*
volatile int time = 0;
volatile int lastPulse = 0;
volatile int pinCurrentState = 0;
volatile int pinLastState = 0;
volatile bool direction = false;  /**< false = Anti-Clockwise, true = Clockwise */

int R, G, B;              /**< Color Sensor RGB Values */
int DATA;                 /**< Color Sensor Data Value */
bool LINE;                /**< Color Sensor Line Detection */
int AVG_IRL, AVG_IRR;     /**< Average IR Sensor Value */
int IRR_LDATA ,IRL_LDATA; /**< IR Sensor Last Values */
int IRR_DATA, IRL_DATA;   /**< IR Sensor Storage Variables */


void goStraight(){
  digitalWrite(M1_CLOCK,HIGH);
  digitalWrite(M2_CLOCK,HIGH);
  digitalWrite(M1_ACLOCK,LOW);
  digitalWrite(M2_ACLOCK,LOW);
}

void goRight(){
  digitalWrite(M1_CLOCK,HIGH);
  digitalWrite(M2_CLOCK,LOW);
  digitalWrite(M1_ACLOCK,LOW);
  digitalWrite(M2_ACLOCK,HIGH);
}

void goLeft(){
  digitalWrite(M1_CLOCK,LOW);
  digitalWrite(M2_CLOCK,HIGH);
  digitalWrite(M1_ACLOCK,HIGH);
  digitalWrite(M2_ACLOCK,LOW);
}

/**
 * Average delay time is around 30 - 40 ms
 */
void readColorSensor(){
  /* S2/S3 Levels define which set of photodiodes we are using 
   * LOW/LOW is for RED 
   * LOW/HIGH is for BLUE 
   * HIGH/HIGH is for GREEN 
   */

  digitalWrite(S2,LOW);        
  digitalWrite(S3,LOW);
  Serial.print("Red value= "); 
  DATA = pulseIn(OUT,LOW);
  delay(10);
  R = DATA;
  Serial.print(R); 

  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  Serial.print("Blue value= ");
  DATA = pulseIn(OUT,LOW);
  delay(10);
  B = DATA;
  Serial.print(B); 

  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  Serial.print("Green value= ");
  DATA = pulseIn(OUT,LOW);
  delay(10);
  G = DATA * 0.65;
  Serial.println(G); 

  if(R<7 && G<7 && B<7){ LINE = false; }
  else{ LINE = true;}
}

void readIRSensor(){
  IRL_DATA = analogRead(IRL) - AVG_IRL;
  IRR_DATA = analogRead(IRR) - AVG_IRR;
  if (IRL_DATA - IRL_LDATA > 100){
    goRight();
  }
  else if (IRR_DATA - IRR_LDATA > 100){
    goLeft();
  }
}

void setup()
{
  Serial.begin(9600);

  /* Defining the PINs Array for Input / Output */
  int outPins[] = {M1_EN, M1_CLOCK, M1_ACLOCK, M2_EN, M2_CLOCK, M2_ACLOCK, IRL_EN, IRR_EN};
  int inPins[] = {IRL, IRR, S0, S1, S2, S3, OUT};

  for(int i = 0; i < sizeof(outPins); i++){
    pinMode(outPins[i], OUTPUT);
  }
  for(int i = 0; i < sizeof(inPins); i++){
    pinMode(inPins[i], OUTPUT);
  }

  /* Read IR Sensor background values */
  digitalWrite(IRL_EN, LOW);
  digitalWrite(IRR_EN, LOW);

  for(int i = 0; i < 20; i++){
    AVG_IRL += analogRead(IRL);
    AVG_IRR += analogRead(IRR);
  }

  AVG_IRL /= 20; AVG_IRR /= 20;
  IRL_LDATA = AVG_IRL; IRR_LDATA = AVG_IRR;
  digitalWrite(IRL_EN, HIGH);
  digitalWrite(IRR_EN, HIGH);

}

void loop()
{
  analogWrite(M1_EN,240);
  analogWrite(M2_EN,255);
  digitalWrite(M1_CLOCK,HIGH);
  digitalWrite(M2_CLOCK,HIGH);
  //Serial.println(time);
  delay(1000);
}

