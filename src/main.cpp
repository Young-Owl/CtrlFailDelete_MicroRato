/**  MicroRato 2023 - Ctrl+Fail+Delete
  *  2022/2023
  *  UA - Universidade de Aveiro
  *  DETI - Departamento de Electrónica, Telecomunicações e Informática
  *  Gonçalo Rodrigues; Marco Santos; João Tavares;
*/

/* Every generation = Add a new line: */
/*          CTRL+FAIL+DELETE          */ // Initial Code for Tests
/*          CTRL+FAIL+DELETE          */ // Iteration 0 - Basic sensor testing
/*          CTRL+FAIL+DELETE          */ // Iteration 1 - Sensors not working; Added 2 sensors + Color Sensor
/*          CTRL+FAIL+DELETE          */ // Iteration 2 - Sensors were actually working, Cathode and Anode were inverted! :D

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_tockn.h>
#include <EasyButton.h>

/* Debug purposes only, remove comment to see the outputs */
#define DEBUG


/* PINs SCL and SDA (I2C Protocol) will be used for the GY-521 Module (Gyroscope)*/
MPU6050 mpu6050(Wire);

/* Defining PINs for the L298N Module 
 * Clockwise = Wheel pushes the robot forward;
 * Anti-Clockwise = Wheel pushes the robot backwards;     */ 
#define M1_EN     2   /**< Motor 1 Speed Control. (PWM)   */
#define M1_CLOCK  53  /**< Motor 1 Clockwise Control.     */
#define M1_ACLOCK 52   /**< Motor 1 Anti-Clockwise Control.*/
#define M2_EN     3   /**< Motor 2 Speed Control. (PWM)   */
#define M2_CLOCK  51  /**< Motor 2 Clockwise Control.     */
#define M2_ACLOCK 50   /**< Motor 2 Anti-Clockwise Control.*/

/* Defining PINs for the 2 side IR sensors */
#define IR_EN 24       /**< Global IR Sensor enable */
#define IRFR A2       /**< Front Right IR Sensor Analog Pin */
#define IRFL A3       /**< Front Left IR Sensor Analog Pin  */
#define IRR A5        /**< Right IR Sensor Analog Pin */
#define IRL A4        /**< Left IR Sensor Analog Pin  */
#define THRESHOLD 500 /**< Decision threshold*/
#define END 0b1111    /**< End of track*/
#define STRAIT 0b0110 /**< Strait path*/
#define LETF_TURN 0b1000 /**< Left turn*/
#define RIGT_TURN 0b0001 /**< Right turn*/

/* Defining PINs for the START and STOP Buttons */
#define START 28      /**< Start Button Pin */
#define STOP 29       /**< Stop Button Pin  */

int AVG_IRL, AVG_IRR, AVG_IRFL, AVG_IRFR;         /**< Average IR Sensor Value */
int IRR_LDATA ,IRL_LDATA, IRFR_LDATA, IRFL_LDATA; /**< IR Sensor Last Values */
int IRR_DATA, IRL_DATA, IRFR_DATA, IRFL_DATA;     /**< IR Sensor Storage Variables */
int IR_DATA;
int M1_SPEED = 0, M2_SPEED = 0;                   /**< Motor Speed Variables */
EasyButton startButton(START, 30, false, false);                      /**< Start Button Object */
EasyButton stopButton(STOP, 30, false, false);                        /**< Stop Button Object */
int startState = 0, stopState = 0;                /**< Button State Variable */
int lstartState = 0, lstopState = 0;              /**< Last Button State Variable */
int skrt;                                         /**< Flag, running (change name)*/


void goStraight(){
  M1_SPEED = 225;
  M2_SPEED = 255;
  analogWrite(M1_EN,M1_SPEED);
  analogWrite(M2_EN,M2_SPEED);
  digitalWrite(M1_CLOCK,HIGH);
  digitalWrite(M2_CLOCK,HIGH);
  digitalWrite(M1_ACLOCK,LOW);
  digitalWrite(M2_ACLOCK,LOW);
}

void goRight(){
  M1_SPEED = 225;
  M2_SPEED = 255;
  analogWrite(M1_EN,M1_SPEED);
  analogWrite(M2_EN,M2_SPEED);
  digitalWrite(M1_CLOCK,HIGH);
  digitalWrite(M2_CLOCK,LOW);
  digitalWrite(M1_ACLOCK,LOW);
  digitalWrite(M2_ACLOCK,HIGH);
}

void goLeft(){
  M1_SPEED = 225;
  M2_SPEED = 255;
  analogWrite(M1_EN,M1_SPEED);
  analogWrite(M2_EN,M2_SPEED);
  digitalWrite(M1_CLOCK,LOW);
  digitalWrite(M2_CLOCK,HIGH);
  digitalWrite(M1_ACLOCK,HIGH);
  digitalWrite(M2_ACLOCK,LOW);
}

void stop(){
  M1_SPEED = 0;
  M2_SPEED = 0;
  analogWrite(M1_EN,M1_SPEED);
  analogWrite(M2_EN,M2_SPEED);
  digitalWrite(M1_CLOCK,LOW);
  digitalWrite(M2_CLOCK,LOW);
  digitalWrite(M1_ACLOCK,LOW);
  digitalWrite(M2_ACLOCK,LOW);
}

void readIRSensor(){
  IRL_DATA = analogRead(IRL) - AVG_IRL;
  IRR_DATA = analogRead(IRR) - AVG_IRR;
  IRFL_DATA = analogRead(IRFL) - AVG_IRFL;
  IRFR_DATA = analogRead(IRFR) - AVG_IRFR;

  if(abs(IRL_DATA)< 500) IRL_DATA= 0;
  else IRL_DATA= 1;

  if(abs(IRR_DATA)< 500) IRR_DATA= 0;
  else IRR_DATA= 1;

  if(abs(IRFL_DATA)< 500) IRFL_DATA= 0;
  else IRFL_DATA= 1;

  if(abs(IRFR_DATA)< 500) IRFR_DATA= 0;
  else IRFR_DATA= 1;

  IR_DATA= (IRL_DATA<< 3) + (IRFL_DATA<< 2)+ (IRFR_DATA<< 1) +IRR_DATA;

  /*
  if (IRL_DATA - IRL_LDATA > 100){
    goLeft();
  }
  else if (IRR_DATA - IRR_LDATA > 100){
    goRight();
  }
  */

  if (IR_DATA== LETF_TURN){
    goLeft();
  }
  else if (IR_DATA== RIGT_TURN){
    goRight();
  }
  else if(IR_DATA== STRAIT){  // might not be needed
    goStraight();
  }
  else if(IR_DATA== STRAIT){
    stop();
    skrt= 0;
  }

  IRL_LDATA = IRL_DATA;
  IRR_LDATA = IRR_DATA;
  IRFL_LDATA = IRFL_DATA;
  IRFR_LDATA = IRFR_DATA;


  #ifdef DEBUG
  /*
    Serial.print("IRL: ");
    Serial.print(IRL_DATA);
    Serial.print(" IRR: ");
    Serial.print(IRR_DATA);
    Serial.print(" IRFL: ");
    Serial.print(IRFL_DATA);
    Serial.print(" IRFR: ");
    Serial.println(IRFR_DATA);
  */
    Serial.println(IR_DATA, BIN);
  #endif
}

void setup()
{
  Serial.begin(9600);

  /* Defining the PINs Array for Input / Output */
  int outPins[] = {M1_EN, M1_CLOCK, M1_ACLOCK, M2_EN, M2_CLOCK, M2_ACLOCK, IR_EN};
  int inPins[] = {IRL, IRR, IRFL, IRFR, START, STOP};
  int inPins_size = (sizeof(inPins)/sizeof(inPins[0]));
  int outPins_size = (sizeof(outPins)/sizeof(outPins[0]));

  for(int i = 0; i < outPins_size; i++){
    pinMode(outPins[i], OUTPUT);
  }
  for(int i = 0; i < inPins_size; i++){
    pinMode(inPins[i], OUTPUT);
  }
  pinMode(IRL, INPUT);
  pinMode(IRR, INPUT);
  pinMode(IRFL, INPUT);
  pinMode(IRFR, INPUT);
  
  digitalWrite(IR_EN, LOW);

  for(int i = 0; i < 20; i++){
    AVG_IRL += analogRead(IRL);
    AVG_IRR += analogRead(IRR);
    AVG_IRFL += analogRead(IRFL);
    AVG_IRFR += analogRead(IRFR);
    delay(20);
  }

  AVG_IRL /= 20; AVG_IRR /= 20; AVG_IRFL /= 20; AVG_IRFR /= 20;
  IRL_LDATA = AVG_IRL; IRR_LDATA = AVG_IRR; IRFL_LDATA = AVG_IRFL; IRFR_LDATA = AVG_IRFR;
  digitalWrite(IR_EN, HIGH);

  analogWrite(M1_EN,0);
  analogWrite(M2_EN,0);
  digitalWrite(M1_CLOCK,LOW);
  digitalWrite(M2_CLOCK,LOW);
  digitalWrite(M1_ACLOCK,LOW);
  digitalWrite(M2_ACLOCK,LOW);
  
  startButton.begin();
  stopButton.begin();

  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets();

  #ifdef DEBUG
    Serial.println("Calibration is completed.");
    Serial.println("IR Sensors Background noise: IRL: " + String(AVG_IRL) + 
                                               " IRR: " + String(AVG_IRR) + 
                                               " IRFL: " + String(AVG_IRFL) + 
                                               " IRFR: " + String(AVG_IRFR));
  #endif
}

void loop()
{
  startState = startButton.read();
  stopState = stopButton.read();

  if(startState != lstartState){
    if(startState == 1){
      goStraight();
      skrt= 1;    
    }
  }

  if(stopState != lstopState){
    if(stopState == 1){
      stop();
      skrt= 0;
    }
  }

  if(skrt== 1) readIRSensor();

  lstartState = startState;
  lstopState = stopState;
  delay(10);
}


