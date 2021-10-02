//***********************************************************************
//  MiniHex IR control Program
//  Code for Arduino Mega or Mega Pro Mini
//  by Francisco Carabaza (oct 2021)

//  IK and Hexapod gait references:
//  https://markwtech.com/robots/hexapod/
//  https://www.projectsofdan.com/?cat=4
//  http://www.gperco.com/2015/06/hex-inverse-kinematics.html
//  https://www.robotshop.com/community/forum/t/inverse-kinematic-equations-for-lynxmotion-3dof-legs/21336
//  http://arduin0.blogspot.com/2012/01/inverse-kinematics-ik-implementation.html?utm_source=rb-community&utm_medium=forum&utm_campaign=inverse-kinematic-equations-for-lynxmotion-3dof-legs
//***********************************************************************

#include <Arduino.h>
#include <Servo.h>
#include <math.h>
#define DECODE_NEC                  //Cheap IR remote protocol decoding
#include "PinDefinitionsAndMore.h"  // Define macros for input and output pin etc.
#include "IRremote.h"
#define DEATHBAND 5
#define SMOOTH_FACTOR 0.05
#define SMOOTH_PREVIOUS_FACTOR 0.95
#define HEAD_SMOOTH_FACTOR 0.06
#define HEAD_SMOOTH_PREVIOUS_FACTOR 0.94
#include "Config_MiniHex.hpp"


const long A12DEG = 209440;   //12 degrees in radians x 1,000,000 (12 grados en millonésimas de radián)
const long A18DEG = 314160;   //18 degrees in radians x 1,000,000 (20 grados en millonésimas de radián)
const long A30DEG = 523599;   //30 degrees in radians x 1,000,000 (30 grados en millonésimas de radián)
const int FRAME_TIME_MS = 20; //frame time (20msec = 50Hz)

// pata 1 delantera derecha
// pata 2 media derecha
// pata 3 trasera derecha
// pata 4 trasera izquierda
// pata 5 media izquierda
// pata 6 delantera izquierda

//***********************************************************************
// Variable Declarations
//***********************************************************************
unsigned long currentTime;            //frame timer variables
unsigned long previousTime;

int temp;                             //mode and control variables
int mode;
int gait;
int gait_speed;
int reset_position;
int capture_offsets;

float L0, L3;                         //inverse kinematics variables
float gamma_femur;
float phi_tibia, phi_femur;
float theta_tibia, theta_femur, theta_coxa;

int leg1_IK_control, leg6_IK_control; //leg lift mode variables
float leg1_coxa, leg1_femur, leg1_tibia;
float leg6_coxa, leg6_femur, leg6_tibia;

int leg_num;                          //positioning and walking variables
int z_height_LED_color;
int totalX, totalY, totalZ;
int tick, numTicks;
int duration = 1080;
int z_height_left, z_height_right;
int commandedX, commandedY, commandedR;
int translateX, translateY, translateZ;
float step_height_multiplier;
float strideX, strideY, strideR;
float sinRotX, sinRotY, sinRotZ;
float cosRotX, cosRotY, cosRotZ;
float rotOffsetX, rotOffsetY, rotOffsetZ;
float amplitudeX, amplitudeY, amplitudeZ;
float offset_X[6], offset_Y[6], offset_Z[6];
float current_X[6], current_Y[6], current_Z[6];

int tripod_case[6]   = {1, 2, 1, 2, 1, 2}; //for tripod gait walking
int ripple_case[6]   = {2, 6, 4, 1, 3, 5}; //for ripple gait
int wave_case[6]     = {1, 2, 3, 4, 5, 6}; //for wave gait
int tetrapod_case[6] = {1, 3, 2, 1, 2, 3}; //for tetrapod gait

int RX = 127;           //X right control value
float RX_Previous = 127;
float RX_Smoothed = 127;

int RY = 127; //Y right control value
float RY_Previous = 127;
float RY_Smoothed = 127;

int RZ = 127; //Rotatión value
float RZ_Previous = 127;
float RZ_Smoothed = 127;

int LX = 127; //X left control value
float LX_Previous = 127;
float LX_Smoothed = 127;

int LY = 127; //Y left control value
float LY_Previous = 127;
float LY_Smoothed = 127;

int LZ = 127; //Rotatión value from left
float LZ_Previous = 127;
float LZ_Smoothed = 127;

//Head variables
int HX = 90; //pan head angle
float HX_Previous = 90;
float HX_Smoothed = 90;

int HY = 90; //tilt head angle
float HY_Previous = 90;
float HY_Smoothed = 90;

#include "IR_Control.hpp"

//***********************************************************************
// Object Declarations
//***********************************************************************
Servo coxa1_servo;
Servo femur1_servo;
Servo tibia1_servo;
Servo coxa2_servo;
Servo femur2_servo;
Servo tibia2_servo;
Servo coxa3_servo;
Servo femur3_servo;
Servo tibia3_servo;
Servo coxa4_servo;
Servo femur4_servo;
Servo tibia4_servo;
Servo coxa5_servo;
Servo femur5_servo;
Servo tibia5_servo;
Servo coxa6_servo;
Servo femur6_servo;
Servo tibia6_servo;

//************************************************************************************
// Setup
//************************************************************************************
void setup()
{
  Serial.begin(115200);   //Serial para debug en el PC

  //attach servos
  coxa1_servo.attach(COXA1_SERVO, MS_SERVO_MIN, MS_SERVO_MAX);
  femur1_servo.attach(FEMUR1_SERVO, MS_SERVO_MIN, MS_SERVO_MAX);
  tibia1_servo.attach(TIBIA1_SERVO, MS_SERVO_MIN, MS_SERVO_MAX);
  coxa2_servo.attach(COXA2_SERVO, MS_SERVO_MIN, MS_SERVO_MAX);
  femur2_servo.attach(FEMUR2_SERVO, MS_SERVO_MIN, MS_SERVO_MAX);
  tibia2_servo.attach(TIBIA2_SERVO, MS_SERVO_MIN, MS_SERVO_MAX);
  coxa3_servo.attach(COXA3_SERVO, MS_SERVO_MIN, MS_SERVO_MAX);
  femur3_servo.attach(FEMUR3_SERVO, MS_SERVO_MIN, MS_SERVO_MAX);
  tibia3_servo.attach(TIBIA3_SERVO, MS_SERVO_MIN, MS_SERVO_MAX);
  coxa4_servo.attach(COXA4_SERVO, MS_SERVO_MIN, MS_SERVO_MAX);
  femur4_servo.attach(FEMUR4_SERVO, MS_SERVO_MIN, MS_SERVO_MAX);
  tibia4_servo.attach(TIBIA4_SERVO, MS_SERVO_MIN, MS_SERVO_MAX);
  coxa5_servo.attach(COXA5_SERVO, MS_SERVO_MIN, MS_SERVO_MAX);
  femur5_servo.attach(FEMUR5_SERVO, MS_SERVO_MIN, MS_SERVO_MAX);
  tibia5_servo.attach(TIBIA5_SERVO, MS_SERVO_MIN, MS_SERVO_MAX);
  coxa6_servo.attach(COXA6_SERVO, MS_SERVO_MIN, MS_SERVO_MAX);
  femur6_servo.attach(FEMUR6_SERVO, MS_SERVO_MIN, MS_SERVO_MAX);
  tibia6_servo.attach(TIBIA6_SERVO, MS_SERVO_MIN, MS_SERVO_MAX);

  head_pan_servo.attach(HEAD_PAN_SERVO, MS_SERVO_MIN, MS_SERVO_MAX);
  head_pan_servo.write(90);
  head_tilt_servo.attach(HEAD_TILT_SERVO, MS_SERVO_MIN, MS_SERVO_MAX);
  head_tilt_servo.write(90);

  Setup_IR_Receiver();

  //clear offsets
  for (leg_num = 0; leg_num < 6; leg_num++)
  {
    offset_X[leg_num] = 0.0;
    offset_Y[leg_num] = 0.0;
    offset_Z[leg_num] = 0.0;
  }

  capture_offsets = false;
  step_height_multiplier = 2.0; //Con este valor se controla la altura del paso

  //initialize mode and gait variables
  mode = 1; //1 walk, 2 translate control, 3 rotate control, 4 oneleglift, 99 set all 90
  gait = 2; //gate 0 tripod_gait, 1 wave gait, 2 ripple gait, 3 tetrapod gait
  gait_speed = 0;
  reset_position = true;
  leg1_IK_control = true;
  leg6_IK_control = true;
}

#include "Gate_Tripod.hpp"
#include "Gate_Wave.hpp"
#include "Gate_Ripple.hpp"
#include "Gate_Tetrapod.hpp"

//************************************************************************************
// Loop
//************************************************************************************
void loop()
{
  Decode_IR_Receiver();
  if ((millis() - previousTime) > FRAME_TIME_MS)
  {
    previousTime = millis();
    JamesBrutonSmoothing();
    move_head();
    print_debug();

    //reset legs to home position when commanded
    if (reset_position == true)
    {
      //Serial.println("Reset position");
      for (leg_num = 0; leg_num < 6; leg_num++)
      {
        current_X[leg_num] = HOME_X[leg_num];
        current_Y[leg_num] = HOME_Y[leg_num];
        current_Z[leg_num] = HOME_Z[leg_num];
      }
      reset_position = false;
    }

    //position legs using IK calculations - unless set all to 90 degrees mode
    if (mode < 99)
    {
      for (leg_num = 0; leg_num < 6; leg_num++)
        leg_IK(leg_num, current_X[leg_num] + offset_X[leg_num], current_Y[leg_num] + offset_Y[leg_num], current_Z[leg_num] + offset_Z[leg_num]);
    }

    //reset leg lift first pass flags if needed
    if (mode != 4)
    {
      leg1_IK_control = true;
      leg6_IK_control = true;
    }

    //process modes (mode 0 is default 'home idle' do-nothing mode)
    if (mode == 1)                            //walking mode
    {
      if (gait == 0) tripod_gait();           //walk using gait 0
      if (gait == 1) wave_gait();             //walk using gait 1
      if (gait == 2) ripple_gait();           //walk using gait 2
      if (gait == 3) tetrapod_gait();         //walk using gait 3
    }
    if (mode == 2) translate_control();
    if (mode == 3) rotate_control();          //joystick control y-p-r mode
    if (mode == 99) set_all_90();             //set all servos to 90 degrees mode
  }
}

void JamesBrutonSmoothing() {
  RX_Smoothed = (RX * SMOOTH_FACTOR) + (RX_Previous * SMOOTH_PREVIOUS_FACTOR);
  RX_Previous = RX_Smoothed;

  RY_Smoothed = (RY * SMOOTH_FACTOR) + (RY_Previous * SMOOTH_PREVIOUS_FACTOR);
  RY_Previous = RY_Smoothed;

  RZ_Smoothed = (RZ * SMOOTH_FACTOR) + (RZ_Previous * SMOOTH_PREVIOUS_FACTOR);
  RZ_Previous = RZ_Smoothed;

  LX_Smoothed = (LX * SMOOTH_FACTOR) + (LX_Previous * SMOOTH_PREVIOUS_FACTOR);
  LX_Previous = LX_Smoothed;

  LY_Smoothed = (LY * SMOOTH_FACTOR) + (LY_Previous * SMOOTH_PREVIOUS_FACTOR);
  LY_Previous = LY_Smoothed;

  LZ_Smoothed = (LZ * SMOOTH_FACTOR) + (LZ_Previous * SMOOTH_PREVIOUS_FACTOR);
  LZ_Previous = LZ_Smoothed;

  HX_Smoothed = (HX * HEAD_SMOOTH_FACTOR) + (HX_Previous * HEAD_SMOOTH_PREVIOUS_FACTOR);
  HX_Previous = HX_Smoothed;

  HY_Smoothed = (HY * HEAD_SMOOTH_FACTOR) + (HY_Previous * HEAD_SMOOTH_PREVIOUS_FACTOR);
  HY_Previous = HY_Smoothed;
}


//***********************************************************************
// Leg IK Routine
//***********************************************************************
void leg_IK(int leg_number, float X, float Y, float Z)
{
  //compute target femur-to-toe (L3) length
  L0 = sqrt(sq(X) + sq(Y)) - COXA_LENGTH;
  L3 = sqrt(sq(L0) + sq(Z));

  //process only if reach is within possible range (not too long or too short!)
  if ((L3 < (TIBIA_LENGTH + FEMUR_LENGTH)) && (L3 > (TIBIA_LENGTH - FEMUR_LENGTH)))
  {
    //compute tibia angle
    phi_tibia = acos((sq(FEMUR_LENGTH) + sq(TIBIA_LENGTH) - sq(L3)) / (2 * FEMUR_LENGTH * TIBIA_LENGTH));
    theta_tibia = phi_tibia * RAD_TO_DEG - 23.0 + TIBIA_CAL[leg_number];
    theta_tibia = constrain(theta_tibia, 0.0, 180.0);

    //compute femur angle
    gamma_femur = atan2(Z, L0);
    phi_femur = acos((sq(FEMUR_LENGTH) + sq(L3) - sq(TIBIA_LENGTH)) / (2 * FEMUR_LENGTH * L3));
    theta_femur = (phi_femur + gamma_femur) * RAD_TO_DEG + 14.0 + 90.0 + FEMUR_CAL[leg_number];
    theta_femur = constrain(theta_femur, 0.0, 180.0);

    //compute coxa angle
    theta_coxa = atan2(X, Y) * RAD_TO_DEG + COXA_CAL[leg_number];

    //output to the appropriate leg
    switch (leg_number)
    {
      case 0:
        if (leg1_IK_control == true)                      //flag for IK or manual control of leg
        {
          theta_coxa = theta_coxa + 90.0;                 //compensate for leg mounting
          theta_coxa = constrain(theta_coxa, 0.0, 180.0);
          theta_coxa = map(theta_coxa, 0, 180, 180, 0); //inverted servo
          coxa1_servo.write(int(theta_coxa));
          theta_femur = map(theta_femur, 0, 180, 180, 0); //inverted servo
          femur1_servo.write(int(theta_femur));
          tibia1_servo.write(int(theta_tibia));
        }
        break;
      case 1:
        theta_coxa = theta_coxa + 90.0;                 //compensate for leg mounting
        theta_coxa = constrain(theta_coxa, 0.0, 180.0);
        theta_coxa = map(theta_coxa, 0, 180, 180, 0); //inverted servo
        coxa2_servo.write(int(theta_coxa));
        theta_femur = map(theta_femur, 0, 180, 180, 0); //inverted servo
        femur2_servo.write(int(theta_femur));
        tibia2_servo.write(int(theta_tibia));
        break;
      case 2:
        theta_coxa = theta_coxa + 90.0;                 //compensate for leg mounting
        theta_coxa = constrain(theta_coxa, 0.0, 180.0);
        theta_coxa = map(theta_coxa, 0, 180, 180, 0); //inverted servo
        coxa3_servo.write(int(theta_coxa));
        theta_femur = map(theta_femur, 0, 180, 180, 0); //inverted servo
        femur3_servo.write(int(theta_femur));
        tibia3_servo.write(int(theta_tibia));
        break;
      case 3:
        if (theta_coxa < 0)                               //compensate for leg mounting
          theta_coxa = theta_coxa + 270.0;                // (need to use different
        else                                              //  positive and negative offsets
          theta_coxa = theta_coxa - 90.0;                //  due to atan2 results above!)
        theta_coxa = constrain(theta_coxa, 0.0, 180.0);
        theta_coxa = map(theta_coxa, 0, 180, 180, 0); //inverted servo
        coxa4_servo.write(int(theta_coxa));
        femur4_servo.write(int(theta_femur));
        theta_tibia = map(theta_tibia, 0, 180, 180, 0); //inverted servo
        tibia4_servo.write(int(theta_tibia));
        break;
      case 4:
        if (theta_coxa < 0)                               //compensate for leg mounting
          theta_coxa = theta_coxa + 270.0;                // (need to use different
        else                                              //  positive and negative offsets
          theta_coxa = theta_coxa - 90.0;                 //  due to atan2 results above!)
        theta_coxa = constrain(theta_coxa, 0.0, 180.0);
        theta_coxa = map(theta_coxa, 0, 180, 180, 0); //inverted servo
        coxa5_servo.write(int(theta_coxa));
        femur5_servo.write(int(theta_femur));
        theta_tibia = map(theta_tibia, 0, 180, 180, 0); //inverted servo
        tibia5_servo.write(int(theta_tibia));
        break;
      case 5:
        if (leg6_IK_control == true)                      //flag for IK or manual control of leg
        {
          if (theta_coxa < 0)                             //compensate for leg mounting
            theta_coxa = theta_coxa + 270.0;              // (need to use different
          else                                            //  positive and negative offsets
            theta_coxa = theta_coxa - 90.0;               //  due to atan2 results above!)
          theta_coxa = constrain(theta_coxa, 0.0, 180.0);
          theta_coxa = map(theta_coxa, 0, 180, 180, 0); //inverted servo
          coxa6_servo.write(int(theta_coxa));
          femur6_servo.write(int(theta_femur));
          theta_tibia = map(theta_tibia, 0, 180, 180, 0); //inverted servo
          tibia6_servo.write(int(theta_tibia));
        }
        break;
    }
  }
}


//***********************************************************************
// Compute walking stride lengths
//***********************************************************************
void compute_strides()
{
  //compute stride lengths
  strideX = STRIDE_X_MULTIPLIER * commandedX / 127;
  strideY = STRIDE_Y_MULTIPLIER * commandedY / 127;
  strideR = STRIDE_R_MULTIPLIER * commandedR / 127;

  //compute rotation trig
  sinRotZ = sin(radians(strideR));
  cosRotZ = cos(radians(strideR));
}


//***********************************************************************
// Compute walking amplitudes
//***********************************************************************
void compute_amplitudes()
{
  //compute total distance from center of body to toe
  totalX = HOME_X[leg_num] + BODY_X[leg_num];
  totalY = HOME_Y[leg_num] + BODY_Y[leg_num];

  //compute rotational offset
  rotOffsetX = totalY * sinRotZ + totalX * cosRotZ - totalX;
  rotOffsetY = totalY * cosRotZ - totalX * sinRotZ - totalY;

  //compute X and Y amplitude and constrain to prevent legs from crashing into each other
  amplitudeX = ((strideX + rotOffsetX) / 2.0);
  amplitudeY = ((strideY + rotOffsetY) / 2.0);
  amplitudeX = constrain(amplitudeX, -50, 50); //originalmente -50 a 50
  amplitudeY = constrain(amplitudeY, -50, 50); //originalmente -50 a 50

  //compute Z amplitude
  if (abs(strideX + rotOffsetX) > abs(strideY + rotOffsetY))
    amplitudeZ = step_height_multiplier * (strideX + rotOffsetX) / 4.0;
  else
    amplitudeZ = step_height_multiplier * (strideY + rotOffsetY) / 4.0;
}

//***********************************************************************
// Body translate with controller (xyz axes)
//***********************************************************************
void translate_control()
{
  //compute X direction move
  translateX = map(RY_Smoothed, 255, 0, -1 * TRAVEL, 1 * TRAVEL);
  for (leg_num = 0; leg_num < 6; leg_num++)
    current_X[leg_num] = HOME_X[leg_num] + translateX;

  //compute Y direction move
  translateY = map(RX_Smoothed, 0, 255, 1 * TRAVEL, -1 * TRAVEL);
  for (leg_num = 0; leg_num < 6; leg_num++)
    current_Y[leg_num] = HOME_Y[leg_num] + translateY;

  //compute Z direction move
  translateZ = LX_Smoothed;
  if (translateZ > 127)
    translateZ = map(translateZ, 128, 255, 0, TRAVEL);
  else
    translateZ = map(translateZ, 0, 127, -1 * TRAVEL, 0);
  for (leg_num = 0; leg_num < 6; leg_num++)
    current_Z[leg_num] = HOME_Z[leg_num] + translateZ;

  //lock in offsets if commanded
  if (capture_offsets == true)
  {
    for (leg_num = 0; leg_num < 6; leg_num++)
    {
      offset_X[leg_num] = offset_X[leg_num] + translateX;
      offset_Y[leg_num] = offset_Y[leg_num] + translateY;
      offset_Z[leg_num] = offset_Z[leg_num] + translateZ;
      current_X[leg_num] = HOME_X[leg_num];
      current_Y[leg_num] = HOME_Y[leg_num];
      current_Z[leg_num] = HOME_Z[leg_num];
    }
  }

  //if offsets were commanded, exit current mode
  if (capture_offsets == true)
  {
    capture_offsets = false;
    mode = 0;
  }
}


//***********************************************************************
// Body rotate with controller (xyz axes)
//***********************************************************************
void rotate_control()
{
  //compute rotation sin/cos values using controller inputs
  sinRotX = sin((map(RX_Smoothed, 0, 255, A12DEG, -A12DEG)) / 1000000.0);
  cosRotX = cos((map(RX_Smoothed, 0, 255, A12DEG, -A12DEG)) / 1000000.0);
  sinRotY = sin((map(RY_Smoothed, 255, 0, A12DEG, -A12DEG)) / 1000000.0);
  cosRotY = cos((map(RY_Smoothed, 255, 0, A12DEG, -A12DEG)) / 1000000.0);
  sinRotZ = sin((map(LX_Smoothed, 0, 255, -A30DEG, A30DEG)) / 1000000.0);
  cosRotZ = cos((map(LX_Smoothed, 0, 255, -A30DEG, A30DEG)) / 1000000.0);

  //compute Z direction move
  translateZ = LY_Smoothed;
  if (translateZ > 127)
    translateZ = map(translateZ, 128, 255, 0, 1 * TRAVEL);
  else
    translateZ = map(translateZ, 0, 127, -1 * TRAVEL, 0);

  for (int leg_num = 0; leg_num < 6; leg_num++)
  {
    //compute total distance from center of body to toe
    totalX = HOME_X[leg_num] + BODY_X[leg_num];
    totalY = HOME_Y[leg_num] + BODY_Y[leg_num];
    totalZ = HOME_Z[leg_num] + BODY_Z[leg_num];

    //perform 3 axis rotations
    rotOffsetX =  totalX * cosRotY * cosRotZ + totalY * sinRotX * sinRotY * cosRotZ + totalY * cosRotX * sinRotZ - totalZ * cosRotX * sinRotY * cosRotZ + totalZ * sinRotX * sinRotZ - totalX;
    rotOffsetY = -totalX * cosRotY * sinRotZ - totalY * sinRotX * sinRotY * sinRotZ + totalY * cosRotX * cosRotZ + totalZ * cosRotX * sinRotY * sinRotZ + totalZ * sinRotX * cosRotZ - totalY;
    rotOffsetZ =  totalX * sinRotY - totalY * sinRotX * cosRotY + totalZ * cosRotX * cosRotY - totalZ;

    // Calculate foot positions to achieve desired rotation
    current_X[leg_num] = HOME_X[leg_num] + rotOffsetX;
    current_Y[leg_num] = HOME_Y[leg_num] + rotOffsetY;
    current_Z[leg_num] = HOME_Z[leg_num] + rotOffsetZ + translateZ;

    //lock in offsets if commanded
    if (capture_offsets == true)
    {
      offset_X[leg_num] = offset_X[leg_num] + rotOffsetX;
      offset_Y[leg_num] = offset_Y[leg_num] + rotOffsetY;
      offset_Z[leg_num] = offset_Z[leg_num] + rotOffsetZ + translateZ;
      current_X[leg_num] = HOME_X[leg_num];
      current_Y[leg_num] = HOME_Y[leg_num];
      current_Z[leg_num] = HOME_Z[leg_num];
    }
  }

  //if offsets were commanded, exit current mode
  if (capture_offsets == true)
  {
    capture_offsets = false;
    mode = 0;
  }
}

void move_head() //Update head servos
{
  head_pan_servo.write(HX_Smoothed);
  head_tilt_servo.write(HY_Smoothed);
}

//***********************************************************************
// Set all servos to 90 degrees
// Note: this is useful for calibration/alignment of the servos
// i.e: set COXA_CAL[6], FEMUR_CAL[6], and TIBIA_CAL[6] values in
//      constants section above so all angles appear as 90 degrees
//***********************************************************************
void set_all_90()
{
  coxa1_servo.write(90 + COXA_CAL[0]);
  femur1_servo.write(90 + FEMUR_CAL[0]);
  tibia1_servo.write(90 + TIBIA_CAL[0]);

  coxa2_servo.write(90 + COXA_CAL[1]);
  femur2_servo.write(90 + FEMUR_CAL[1]);
  tibia2_servo.write(90 + TIBIA_CAL[1]);

  coxa3_servo.write(90 + COXA_CAL[2]);
  femur3_servo.write(90 + FEMUR_CAL[2]);
  tibia3_servo.write(90 + TIBIA_CAL[2]);

  coxa4_servo.write(90 + COXA_CAL[3]);
  femur4_servo.write(90 + FEMUR_CAL[3]);
  tibia4_servo.write(90 + TIBIA_CAL[3]);

  coxa5_servo.write(90 + COXA_CAL[4]);
  femur5_servo.write(90 + FEMUR_CAL[4]);
  tibia5_servo.write(90 + TIBIA_CAL[4]);

  coxa6_servo.write(90 + COXA_CAL[5]);
  femur6_servo.write(90 + FEMUR_CAL[5]);
  tibia6_servo.write(90 + TIBIA_CAL[5]);

  while (true) {
  }
}

void clear_offsets()
{
  for (leg_num = 0; leg_num < 6; leg_num++)
  {
    offset_X[leg_num] = 0.0;
    offset_Y[leg_num] = 0.0;
    offset_Z[leg_num] = 0.0;
  }
}

//***********************************************************************
// Print Debug Data
//***********************************************************************
void print_debug()
{
  //output variable controls
  Serial.print("RX:");
  Serial.print(RX);
  Serial.print("\tRY:");
  Serial.print(RY);
  Serial.print("\tLX:");
  Serial.print(LX);
  Serial.print("\tLY:");
  Serial.print(LY);
  Serial.print("\tMode ");
  Serial.print(mode);
  Serial.print("\tGait ");
  Serial.println(gait);
}
