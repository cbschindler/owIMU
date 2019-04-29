#ifndef __BOARD_H
#define __BOARD_H

/**
\addtogroup BSP
\{
\addtogroup board
\{

\brief Cross-platform declaration "board" bsp module.

\author Thomas Watteyne <watteyne@eecs.berkeley.edu>, February 2012.
*/

#include "board_info.h"
#include "toolchain_defs.h"

//=========================== define ==========================================

typedef enum {
   DO_NOT_KICK_SCHEDULER,
   KICK_SCHEDULER,
} kick_scheduler_t;

#define PULSE_TRACK_COUNT 5

#define MIN_SYNC_PERIOD_US 52
#define MAX_SYNC_PERIOD_US 138

#define PI 3.14159265f
#define SWEEP_PERIOD_US 8333.333333f

#define CLOCK_SPEED_MHZ 32.0f

//=========================== typedef =========================================

typedef struct {
   uint32_t                rise;
   uint32_t                fall;
   int                     type; // -1 for unclassified, 0 for Sync, 1 for Horiz, 2 for Vert
} pulse_t;

typedef struct {
	float                    phi;
	float                  theta;
	float                 r_vert;
	float				 r_horiz;
	uint8_t               asn[5];
	int					   valid;
} location_t;

typedef enum {
   Sync, Vert, Horiz,
} Pulses;

//=========================== variables =======================================

/* Declaration of global variables for mattress calibration. */
extern float azimuth;
extern float elevation;
extern float roll_calib;
extern float pitch_calib;

//=========================== prototypes ======================================

void board_init(void);
void board_sleep(void);
void board_reset(void);

/**
\}
\}
*/

#endif
