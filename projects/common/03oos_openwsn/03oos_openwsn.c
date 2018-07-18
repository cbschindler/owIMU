/**
\brief This project runs the full OpenWSN stack.

\author Thomas Watteyne <watteyne@eecs.berkeley.edu>, August 2010
*/

#include "board.h"
#include "scheduler.h"
#include "openstack.h"
#include "opendefs.h"

/*
#include "accel_mimsy.h"
#include "flash_mimsy.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
*/

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <headers/hw_memmap.h>
#include <source/gpio.h>
#include <headers/hw_ioc.h>
#include <source/ioc.h>
#include <source/interrupt.h>
#include <source/adc.h>
#include <source/sys_ctrl.h>
#include <headers/hw_sys_ctrl.h>
#include <source/systick.h>
#include <headers/hw_cctest.h>
#include <headers/hw_rfcore_xreg.h>

int mote_main(void) {
   
   // initialize
   board_init();
   scheduler_init();
   openstack_init();




   
   // start
   scheduler_start();
   return 0; // this line should never be reached
}

void sniffer_setListeningChannel(uint8_t channel){return;}
