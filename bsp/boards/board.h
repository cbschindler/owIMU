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
#define MAX_SAMPLES 200

#define CALIB_DATA_STRUCT_SIZE 60 // FIXME: update for calibration data struct size

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

/*CalibData is a union data structure used to store MIMSY's mattress calibration data in flash. Access the struct
type of this union is used for accessing and setting the data. The uint32 array version
of the struct is used for reading and writing to flash*/
typedef union CalibData {
  struct {
  uint32_t pulse_0s;
  uint32_t pulse_0e;
  uint32_t pulse_1s;
  uint32_t pulse_1e;
} fields;
  struct {
  int32_t pulse_0s;
  int32_t pulse_0e;
  int32_t pulse_1s;
  int32_t pulse_1e;
} signedfields;
uint32_t bits[4];
} CalibData; // FIXME: update for position and IMU data (roll/pitch)

/*This struct is used to keep track of where data was written to. This struct
must be passed to flashWriteCalib where it is updated to include the flash location
of the data. A written data card is passed to flashReadCalib inorder to read the
data from that location*/
typedef struct CalibDataCard{
    uint32_t page;
    uint32_t startTime;
    uint32_t endTime;
} CalibDataCard;

//=========================== variables =======================================

/* Declaration of global variables for mattress calibration. */
extern float azimuth;
extern float elevation;
extern float roll_calib;
extern float pitch_calib;

//=========================== prototypes ======================================

extern void flashWriteCalib(CalibData data[],uint32_t size, uint32_t startPage, int wordsWritten);
extern void flashReadCalib(CalibDataCard card, CalibData *data, uint32_t size);
extern void flashReadCalibSection(CalibDataCard card, CalibData *data, uint32_t size,int wordsRead);

void board_init(void);
void board_sleep(void);
void board_reset(void);

/**
\}
\}
*/

#endif
