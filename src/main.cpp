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
#define M1_EN     2     /**< Motor 1 Speed Control. (PWM)   */
#define M1_CLOCK  53    /**< Motor 1 Clockwise Control.     */
#define M1_ACLOCK 50    /**< Motor 1 Anti-Clockwise Control.*/
#define M2_EN     3     /**< Motor 2 Speed Control. (PWM)   */
#define M2_CLOCK  51    /**< Motor 2 Clockwise Control.     */
#define M2_ACLOCK 52    /**< Motor 2 Anti-Clockwise Control.*/

/* Defining PINs for the 2 side IR sensors */
#define IR_EN 24          /**< Global IR Sensor enable */
#define IRFR A3           /**< Front Right IR Sensor Analog Pin */
#define IRFL A2           /**< Front Left IR Sensor Analog Pin  */
#define IRR A5            /**< Right IR Sensor Analog Pin */
#define IRL A4            /**< Left IR Sensor Analog Pin  */
#define THRESHOLD 500     /**< Decision threshold */
#define END 0b1111                /**< End of track */
#define STRAIGHT 0b0110           /**< Strait path */
#define LEFT_TURN 0b1110          /**< Left turn */
#define RIGHT_TURN 0b0111         /**< Right turn */
#define SLIGHT_LEFT 0b0010        /**< Left path */
#define SLIGHT_RIGHT 0b0100       /**< Right path */
#define BACK 0b0000               /**< Back path */
#define FAST_LEFT 0b1000          /**< Front path */
#define FAST_RIGHT 0b0001         /**< Front path */

/* Defining PINs for the START and STOP Buttons */
#define START 28      /**< Start Button Pin */
#define STOP 29       /**< Stop Button Pin  */

/* Defining PIN for the Led above the head */
#define HEAD_LED 41   /**< Led Pin for visual debugging */

#define MULTIPLIER 0.6    /**< Motor Speed Multiplier */
#define RMOTOR_SPEED 225 * MULTIPLIER  /**< Motor Speed */
#define LMOTOR_SPEED 255 * MULTIPLIER  /**< Motor Speed */


int AVG_IRL, AVG_IRR, AVG_IRFL, AVG_IRFR;         /**< Average IR Sensor Value */
int IRR_LDATA ,IRL_LDATA, IRFR_LDATA, IRFL_LDATA; /**< IR Sensor Last Values */
int IRR_DATA, IRL_DATA, IRFR_DATA, IRFL_DATA;     /**< IR Sensor Storage Variables */
int IR_DATA;                                      /**< IR Sensor Visual Variable */                    
int M1_SPEED = 0, M2_SPEED = 0;                   /**< Motor Speed Variables */
EasyButton startButton(START, 30, false, false);  /**< Start Button Object */
EasyButton stopButton(STOP, 30, false, false);    /**< Stop Button Object */
int startState = 0, stopState = 0;                /**< Button State Variable */
int lstartState = 0, lstopState = 0;              /**< Last Button State Variable */
int run;                                          /**< Flag, running (change name)*/
int ledState = 0;                                 /**< Led State Variable */
int lastAngle = 0;                                /**< Gyroscope Last Angle Variable */
int timeCount = 0;                                /**< Time Counter Variable */

void goBack(){
  digitalWrite(M1_CLOCK,LOW);
  digitalWrite(M2_CLOCK,LOW);
  digitalWrite(M1_ACLOCK,HIGH);
  digitalWrite(M2_ACLOCK,HIGH);
  analogWrite(M1_EN,RMOTOR_SPEED);
  analogWrite(M2_EN,LMOTOR_SPEED);

  #ifdef DEBUG
  Serial.println("BACK");
  #endif
}

void stop(){
  M1_SPEED = 0;
  M2_SPEED = 0;
  digitalWrite(M1_CLOCK,LOW);
  digitalWrite(M2_CLOCK,LOW);
  digitalWrite(M1_ACLOCK,LOW);
  digitalWrite(M2_ACLOCK,LOW);
  analogWrite(M1_EN,LMOTOR_SPEED*0);
  analogWrite(M2_EN,RMOTOR_SPEED*0);

  #ifdef DEBUG
  Serial.println("STOP");
  #endif
}

void goStraight(){
  digitalWrite(M1_CLOCK,HIGH);
  digitalWrite(M2_CLOCK,HIGH);
  digitalWrite(M1_ACLOCK,LOW);
  digitalWrite(M2_ACLOCK,LOW);
  analogWrite(M1_EN,RMOTOR_SPEED);
  analogWrite(M2_EN,LMOTOR_SPEED);

  #ifdef DEBUG
  //Serial.println("STRAIT");
  #endif
}

void goRight(){
  
  stop();
  mpu6050.update();
  lastAngle = mpu6050.getGyroAngleX();
  goBack();
  delay(200);
  stop();
  delay(100);
  digitalWrite(M1_CLOCK,HIGH);
  digitalWrite(M2_CLOCK,LOW);
  digitalWrite(M1_ACLOCK,LOW);
  digitalWrite(M2_ACLOCK,HIGH); 
  analogWrite(M1_EN,RMOTOR_SPEED);
  analogWrite(M2_EN,LMOTOR_SPEED);

  delay(240);

  #ifdef DEBUG
  Serial.println("RIGHT");
  #endif
}

void goLeft(){
  
  stop();
  mpu6050.update();
  lastAngle = mpu6050.getGyroAngleX();
  goBack();
  delay(200);
  stop();
  delay(100);

  digitalWrite(M1_CLOCK,LOW);
  digitalWrite(M2_ACLOCK,LOW);
  digitalWrite(M2_CLOCK,HIGH);
  digitalWrite(M1_ACLOCK,HIGH);
  analogWrite(M1_EN,RMOTOR_SPEED);
  analogWrite(M2_EN,LMOTOR_SPEED);


  delay(240);

  /*while(lastAngle - mpu6050.getGyroAngleX() < abs(90)){
    mpu6050.update();
    #ifdef DEBUG
      Serial.println(mpu6050.getGyroAngleX());
    #endif
    delay(50);
  }*/

  #ifdef DEBUG
  Serial.println("LEFT");
  #endif
}

void goSlightLeft(){

  digitalWrite(M1_CLOCK,HIGH);
  digitalWrite(M2_ACLOCK,LOW);
  digitalWrite(M2_CLOCK,HIGH);
  digitalWrite(M1_ACLOCK,LOW);
  analogWrite(M1_EN,RMOTOR_SPEED * 0.8);
  analogWrite(M2_EN,LMOTOR_SPEED * 0.6);

  #ifdef DEBUG
  Serial.println("SLIGHT LEFT");
  #endif
}

void goSlightRight(){
  
  digitalWrite(M1_CLOCK,HIGH);
  digitalWrite(M2_ACLOCK,LOW);
  digitalWrite(M2_CLOCK,HIGH);
  digitalWrite(M1_ACLOCK,LOW);
  analogWrite(M1_EN,RMOTOR_SPEED*0.6);
  analogWrite(M2_EN,LMOTOR_SPEED*0.8);
  
    #ifdef DEBUG
    Serial.println("SLIGHT RIGHT");
    #endif
}



void goFullLeft(){
  digitalWrite(M1_CLOCK,HIGH);
  digitalWrite(M2_CLOCK,HIGH);
  digitalWrite(M1_ACLOCK,LOW);
  digitalWrite(M2_ACLOCK,LOW);
  analogWrite(M1_EN,RMOTOR_SPEED*0.8);
  analogWrite(M2_EN,LMOTOR_SPEED*0.5);

  #ifdef DEBUG
  Serial.println("FULL LEFT");
  #endif
}

void goFullRight(){
  digitalWrite(M1_CLOCK,HIGH);
  digitalWrite(M2_CLOCK,HIGH);
  digitalWrite(M1_ACLOCK,LOW);
  digitalWrite(M2_ACLOCK,LOW);
  analogWrite(M1_EN,RMOTOR_SPEED*0.5);
  analogWrite(M2_EN,LMOTOR_SPEED*0.8);

  #ifdef DEBUG
  Serial.println("FULL RIGHT");
  #endif
}

void turn(){
  digitalWrite(M1_CLOCK,LOW);
  digitalWrite(M2_CLOCK,HIGH);
  digitalWrite(M1_ACLOCK,HIGH);
  digitalWrite(M2_ACLOCK,LOW);
  analogWrite(M1_EN,RMOTOR_SPEED);
  analogWrite(M2_EN,LMOTOR_SPEED);

  #ifdef DEBUG
  Serial.println("TURN");
  #endif
  delay(400);
}

void readIRSensor(){

  IRL_DATA = analogRead(IRL);
  IRR_DATA = analogRead(IRR);
  IRFL_DATA = analogRead(IRFL);
  IRFR_DATA = analogRead(IRFR);

  if(IRL_DATA < 500) IRL_DATA= 0;
  else IRL_DATA= 1;

  if(IRR_DATA < 500) IRR_DATA= 0;
  else IRR_DATA= 1;

  if(IRFL_DATA < 500) IRFL_DATA= 0;
  else IRFL_DATA= 1;

  if(IRFR_DATA < 500) IRFR_DATA= 0;
  else IRFR_DATA= 1;

  IR_DATA = IRR_DATA + (IRFR_DATA << 1) + (IRFL_DATA << 2) + (IRL_DATA << 3);

  switch(IR_DATA){
    case(0b0110):
      goStraight();
      break;
    case(0b0010):
      goSlightLeft();
      break;
    case(0b0100):
      goSlightRight();
      break;
    case(0b0001):
      goFullLeft();
      break;
    case(0b1000):
      goFullRight();
      break; 
    case(0b0000):
      turn();
      break;
    case(0b0111):
      goRight();
      break;
    case(0b1110):
      goLeft();
      break;
    case(0b0011):
      goSlightRight();
      break;
    case(0b1100):
      goSlightLeft();
      break;
    case(0b1111):
      stop();
      break;
    default:
      stop();
      break;
  }

  IRL_LDATA = IRL_DATA;
  IRR_LDATA = IRR_DATA;
  IRFL_LDATA = IRFL_DATA;
  IRFR_LDATA = IRFR_DATA;


  #ifdef DEBUG
    Serial.println(IR_DATA, BIN);
  #endif
}

void setup()
{
  Serial.begin(9600);

  /* Defining the PINs Array for Input / Output */
  int outPins[] = {M1_EN, M1_CLOCK, M1_ACLOCK, M2_EN, M2_CLOCK, M2_ACLOCK, IR_EN, HEAD_LED};
  int inPins[] = {IRL, IRR, IRFL, IRFR, START, STOP};
  int inPins_size = (sizeof(inPins)/sizeof(inPins[0]));
  int outPins_size = (sizeof(outPins)/sizeof(outPins[0]));

  for(int i = 0; i < outPins_size; i++){
    pinMode(outPins[i], OUTPUT);
  }
  for(int i = 0; i < inPins_size; i++){
    pinMode(inPins[i], INPUT);
  }

  AVG_IRFL = 0; AVG_IRFR = 0; AVG_IRL = 0; AVG_IRR = 0;

  digitalWrite(HEAD_LED, HIGH);
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
  
  ledState = 0;
}

void loop()
{
  startState = startButton.read();
  stopState = stopButton.read();
  digitalWrite(HEAD_LED, ledState);

  if(startState != lstartState){
    if(startState == 1){
      run= 1;    
    }
  }

  if(stopState != lstopState){
    if(stopState == 1){
      stop();
      run= 0;
    }
  }

  if(run== 1){
    readIRSensor();
  }
  lstartState = startState;
  lstopState = stopState;
  ledState = !ledState;
  delay(50);
}


