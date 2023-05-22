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

/* IR DATA*/
#define END 0b1111        /**< End of track */
#define STRAIT 0b0110     /**< Strait path */

#define LETF_TURN 0b1110  /**< Left turn */
#define SLIGHT_L  0b0100
#define SLIGHT_L2 0b1100
#define LEFT      0b1000

#define RIGHT_TURN 0b0111  /**< Right turn */
#define SLIGHT_R  0b0010
#define SLIGHT_R2 0b0011
#define RIGHT     0b0001

#define INTERSECTION 0b0000       /**< Intersection turn */

/* Defining PINs for the START and STOP Buttons */
#define START 28      /**< Start Button Pin */
#define STOP 29       /**< Stop Button Pin  */

/* Defining PIN for the Led above the head */
#define HEAD_LED 41   /**< Led Pin for visual debugging */

#define RMOTOR_SPEED 225 /**< Motor Speed */
#define LMOTOR_SPEED 255 /**< Motor Speed */

#define FULL_SPEED  130
#define TURN_H      110
#define TURN_L      90

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
int data;

int readIRSensor(){
  /*
  IRL_DATA = analogRead(IRL) - AVG_IRL;
  IRR_DATA = analogRead(IRR) - AVG_IRR;
  IRFL_DATA = analogRead(IRFL) - AVG_IRFL;
  IRFR_DATA = analogRead(IRFR) - AVG_IRFR;
  */

  IRL_DATA = analogRead(IRL);
  IRR_DATA = analogRead(IRR);
  IRFL_DATA = analogRead(IRFL);
  IRFR_DATA = analogRead(IRFR);

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
    //Serial.println(IR_DATA, BIN);
  #endif
  return IR_DATA;
}

void back(){
  digitalWrite(M1_CLOCK,LOW);
  digitalWrite(M2_CLOCK,LOW);
  digitalWrite(M1_ACLOCK,HIGH);
  digitalWrite(M2_ACLOCK,HIGH);
  analogWrite(M1_EN, FULL_SPEED);
  analogWrite(M2_EN, FULL_SPEED);
  delay(100);
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
  //Serial.println("S");
}

void goStraight(){
  digitalWrite(M1_CLOCK,HIGH);
  digitalWrite(M2_CLOCK,HIGH);
  digitalWrite(M1_ACLOCK,LOW);
  digitalWrite(M2_ACLOCK,LOW);
  analogWrite(M1_EN, FULL_SPEED);
  analogWrite(M2_EN, FULL_SPEED);

  #ifdef DEBUG
  //Serial.println("STRAIT");
  #endif
}

void slightR(){
  digitalWrite(M1_CLOCK,HIGH);
  digitalWrite(M2_CLOCK,HIGH);
  digitalWrite(M1_ACLOCK,LOW);
  digitalWrite(M2_ACLOCK,LOW);
  analogWrite(M1_EN, TURN_L);
  analogWrite(M2_EN, TURN_H);
  delay(10);
  //Serial.println("SR");
}

void goRight(){
  stop();
  delay(30);
  back();
  digitalWrite(M1_CLOCK,HIGH);
  digitalWrite(M2_CLOCK,LOW);
  digitalWrite(M1_ACLOCK,LOW);
  digitalWrite(M2_ACLOCK,HIGH); 
  analogWrite(M1_EN, TURN_H);
  analogWrite(M2_EN, TURN_H);
  //Serial.println("R");
  ledState = 0;
  delay(500);
  //stop()
  back();
  delay(100);
}

void goLeft(){
  stop();
  delay(30);
  back();
  digitalWrite(M1_CLOCK,LOW);
  digitalWrite(M2_ACLOCK,LOW);
  digitalWrite(M2_CLOCK,HIGH);
  digitalWrite(M1_ACLOCK,HIGH);
  analogWrite(M1_EN, TURN_H);
  analogWrite(M2_EN, TURN_H);
 //Serial.println("L");
  ledState = 0;
  delay(500);
  //stop();
  back();
  delay(100);
}

void slightL(){
  digitalWrite(M1_CLOCK,HIGH);
  digitalWrite(M2_CLOCK,HIGH);
  digitalWrite(M1_ACLOCK,LOW);
  digitalWrite(M2_ACLOCK,LOW);
  analogWrite(M1_EN,TURN_H);
  analogWrite(M2_EN,TURN_L);
  //Serial.println("SL");
  delay(10);
}

void rotate(){
  stop();
  delay(30);
  //back();
  digitalWrite(M1_CLOCK,HIGH);
  digitalWrite(M2_CLOCK,LOW);
  digitalWrite(M1_ACLOCK,LOW);
  digitalWrite(M2_ACLOCK,HIGH); 
  analogWrite(M1_EN, TURN_H);
  analogWrite(M2_EN, TURN_H);
  //Serial.println("R");
  ledState = 0;
  delay(500);

  back();
  delay(100);
  //goRight();
  stop();
  delay(30);
  //back();
  digitalWrite(M1_CLOCK,HIGH);
  digitalWrite(M2_CLOCK,LOW);
  digitalWrite(M1_ACLOCK,LOW);
  digitalWrite(M2_ACLOCK,HIGH); 
  analogWrite(M1_EN, TURN_H);
  analogWrite(M2_EN, TURN_H);
  //Serial.println("R");
  ledState = 0;
  delay(500);
  back();
  delay(100);
}

void move(void){
  goStraight();
  delay(50);
  data= readIRSensor();
  if(data== END){
    stop();
    //run= 0;
  }
  else if(data== 0b0000) goRight();
}

void control(){
  ledState = 1;
  data= readIRSensor();
  Serial.println(data, BIN);
  if (IR_DATA== LETF_TURN) goLeft();
  else if (IR_DATA== SLIGHT_L) slightR();
  else if (IR_DATA== SLIGHT_L2) slightR();
  else if (IR_DATA== LEFT) slightR();

  else if (IR_DATA== RIGHT_TURN) goRight();
  else if (IR_DATA== SLIGHT_R) slightL();
  else if (IR_DATA== SLIGHT_R2) slightL();
  else if (IR_DATA== RIGHT) slightL();

  else if(IR_DATA== STRAIT) goStraight();
  else if(IR_DATA== INTERSECTION) rotate();
  else if(IR_DATA== END) stop(); //run= 0;
}

void intrstart(void){
  run= 1;
}
void intrstop(void){
  run= 0; 
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
  //startButton.enableInterrupt(intrstart);
  //stopButton.enableInterrupt(intrstop);

  Wire.begin();
  //mpu6050.begin();
  //mpu6050.calcGyroOffsets();

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
  ledState = 1;
  startState = startButton.read();
  stopState = stopButton.read();
  digitalWrite(HEAD_LED, ledState);


  if(startState != lstartState){
    if(startState == 1){
      goStraight();
      run= 1;    
    }
  }

  if(stopState != lstopState){
    if(stopState == 1){
      stop();
      run= 0;
      Serial.println("Here");
    }
  }
  

  if(run== 1){
    //readIRSensor();
    control();
    
    //rotate();
    /*
    delay(400);
    printf("HERE");
    stop();
    run= 0;
    */

    //goRight();
    //stop();
    //run= 0;
    //mpu6050.update();

    //goLeft();
    //rotate();
    //stop();
    //run= 0;
  }
  lstartState = startState;
  lstopState = stopState;
  //ledState = !ledState;
  //delay(50);
}


