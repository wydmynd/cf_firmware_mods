

#include "deck.h"
#include "system.h"
#include "commander.h"

#include "FreeRTOS.h"
#include "task.h"
//#include "timers.h" //for xtimerstart

#include "debug.h"
#include "log.h"
#include "param.h"

#define DEBUG_MODULE "PUSH"

//static xTimerHandle timer;

static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawrate)
{
  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;


  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = yawrate;


  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  setpoint->velocity.x = vx;
  setpoint->velocity.y = vy;

  setpoint->velocity_body = true;
}

typedef enum {
    idle,
    lowUnlock,
    unlocked,
    stopping
} State;

typedef enum {
    running,
    obstacle,
    scanning,
	forward
} Mode;

static State state = idle;
static Mode mode = running;


static const uint16_t unlockThLow = 100;
static const uint16_t unlockThHigh = 300;
static const uint16_t stoppedTh = 300;

static uint32_t obst_time =0;
static uint32_t scan_time =0;

static uint16_t idUp = 0;
static uint16_t idDown = 0;
static uint16_t idLeft = 0;
static uint16_t idRight = 0;
static uint16_t idFront = 0;
static uint16_t idBack = 0;

static uint16_t up = 0;
static uint16_t down = 0;
static uint16_t front = 0;
static uint16_t back = 0;
static uint16_t left = 0;
static uint16_t right = 0;

static uint16_t up_o = 0;
//static uint16_t down_o = 0;
static uint16_t front_o = 0;
static uint16_t back_o = 0;
static uint16_t left_o = 0;
static uint16_t right_o = 0;

static const float velMax = 0.35f;

static const uint16_t radius = 350;
//static const uint16_t frontCollision = 190;
//static const uint16_t rightCollision = 180;

static const float landing_height = 0.12f;
static const float height_sp = 0.35f;

static float factor = 0;
static float max_bias = 0.25f;
static float bias = 0.0f;
static float yawSpeed = 0;
static float yawFactor = 0.0;
static float scanTimer = 0.0;

static float velSide = 0;
static float velFront = 0;
static float height = 0;
static float l_comp = 0;
static float r_comp = 0;
static float f_comp = 0;
static float b_comp = 0;

#define MAX(a,b) ((a>b)?a:b)
#define MIN(a,b) ((a<b)?a:b)

static void sequenceTask()
{
  static setpoint_t setpoint;

  systemWaitStart();

  vTaskDelay(M2T(2000));

  idUp = logGetVarId("range", "up");
  idDown = logGetVarId("range", "zrange");
  idLeft = logGetVarId("range", "left");
  idRight = logGetVarId("range", "right");
  idFront = logGetVarId("range", "front");
  idBack = logGetVarId("range", "back");

  factor = velMax/radius;
  max_bias = 0.3f;
  yawFactor=65.0/(1500-radius);
  obst_time = xTaskGetTickCount();
  scan_time = xTaskGetTickCount();
  //DEBUG_PRINT("%i", idUp);

  DEBUG_PRINT("Waiting for activation ...\n");

  while(1) {
    vTaskDelay(M2T(40));
    //DEBUG_PRINT(".");
    up = logGetUint(idUp);
    if (state == unlocked) {
      down = logGetUint(idDown);
      left = logGetUint(idLeft);
      right = logGetUint(idRight);
      front = logGetUint(idFront);
      back = logGetUint(idBack);

      left_o = radius - MIN(left, radius);
      right_o = radius - MIN(right, radius);
      l_comp = (-1) * left_o * factor;
      r_comp = right_o * factor;
      velSide = r_comp + l_comp;

      front_o = radius - MIN(front, radius);
      back_o = radius - MIN(back, radius);
      f_comp = (-1) * front_o * factor;
      b_comp = back_o * factor;
      velFront = b_comp + f_comp;

      //up_o = radius - MIN(up, radius);
      height = height_sp; // - up_o/1000.0f;


      yawSpeed = 0;

      if (mode == obstacle ) {
    	  yawSpeed = 60.0;
    	  if (xTaskGetTickCount() - obst_time > 1000 ) {
    		  mode = running;
    	  }
      }

      if (mode == scanning ) {
    	  yawSpeed = -60.0;

    	  if (xTaskGetTickCount() - scan_time > scanTimer ) {
    		  bias=0.2;
    		  mode = forward;
    	  }
      }

      if (mode == forward ) {

    	  if (xTaskGetTickCount() - scan_time > scanTimer+750 ) {
    		  yawSpeed = -10.0;
        	  if (bias < max_bias) {
        		    bias=bias + 0.006f ; // * 1.05;
        	  }
    		  mode = running;
    	  }
      }

      if (mode == running) {
          if (front > radius) {
        	  if (bias < max_bias) {
        		    bias=bias + 0.0055f ; // * 1.05;
        	  }


        	  if (right > radius+150) {
        		  scan_time = xTaskGetTickCount();
        		  //right_o = MIN(right+150, 1000);
        		  //scanTimer = right_o/1000 * 500; //set scan time proportional to distance on right
        		  if (right > 850){
        			  bias=0.05;
        			  scanTimer=600;
        		  }
        		  else {
        			  bias=0.2;
        			  scanTimer=250;
        		  }
        		  mode = scanning;
        	  }
        	  else {
        		  yawSpeed= 0;
        	  }
          }
          else {

        	  obst_time = xTaskGetTickCount();
        	  bias=0.0;
        	  mode = obstacle;

          }
      }


      velFront += bias;


      //DEBUG_PRINT("collision f=%i, l=%i, r=%i, f_vel=%f, yaw=%f\n", front, left, right, velFront, yawSpeed);

      if (1) {
        setHoverSetpoint(&setpoint, velFront, velSide, height, yawSpeed);
        commanderSetSetpoint(&setpoint, 3);
      }

      if (up < 80 && up > 0) {
        state = stopping;
        DEBUG_PRINT("STOPPED BY UP SENSOR\n");
      }

    } else {

      if (state == stopping && up > stoppedTh) {
        //DEBUG_PRINT("%i", up);
        //state = idle; //prevent from unlocking after stop
        DEBUG_PRINT("STOP\n");
      }

      if (up < unlockThLow && state == idle && up > 0.001f) {
        DEBUG_PRINT("Waiting for hand to be removed!\n");
        state = lowUnlock;
      }

      if (up > unlockThHigh && state == lowUnlock) {
        DEBUG_PRINT("Unlocked!\n");
        state = unlocked;
      }

      if (state == idle || state == stopping) {
        memset(&setpoint, 0, sizeof(setpoint_t));
        commanderSetSetpoint(&setpoint, 3);
      }
    }
  }
}

static void sequenceInit()
{
  xTaskCreate(sequenceTask, "sequence", 2*configMINIMAL_STACK_SIZE, NULL,
              /*priority*/3, NULL);
}

static bool sequenceTest()
{
  return true;
}

const DeckDriver sequence_deck = {
  .vid = 0,
  .pid = 0,
  .name = "bcPush",

  .usedGpio = 0,  // FIXME: set the used pins

  .init = sequenceInit,
  .test = sequenceTest,
};



DECK_DRIVER(sequence_deck);

PARAM_GROUP_START(deck)
PARAM_ADD(PARAM_FLOAT, height, &height_sp)
PARAM_GROUP_STOP(deck)
