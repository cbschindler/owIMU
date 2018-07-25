#include "opendefs.h"
#include "uinject.h"
#include "openqueue.h"
#include "openserial.h"
#include "packetfunctions.h"
#include "scheduler.h"
#include "IEEE802154E.h"
#include "idmanager.h"
#include "accel_mimsy.h"
#include "flash_mimsy.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "ml_math_func.h"
#include <math.h>




#include "i2c.h"
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
// #####################################################################################


// ########## DEFINES FOR TEMPERATURE SENSOR ##################
#define CONST 0.58134 //(VREF / 2047) = (1190 / 2047), VREF from Datasheet
#define OFFSET_DATASHEET_25C 827 // 1422*CONST, from Datasheet
#define TEMP_COEFF (CONST * 4.2) // From Datasheet
#define OFFSET_0C (OFFSET_DATASHEET_25C - (25 * TEMP_COEFF))

//=========================== variables =======================================

uinject_vars_t uinject_vars;

static const uint8_t uinject_payload[]    = "abcabca"; //was "uinject", i changed it so it doesnt crash parserdata.py;
static const uint8_t uinject_dst_addr[]   = {
   0xbb, 0xbb, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01
};

//=========================== prototypes ======================================

void uinject_timer_cb(opentimers_id_t id);
void uinject_task_cb(void);
void quat2euler(long * quat, float * yaw, float * pitch, float * roll);

//=========================== public ==========================================

void uinject_init() {
  	mimsyIMUInit();

    mpu_set_sensors(INV_XYZ_GYRO|INV_XYZ_ACCEL); // turn on sensors
    mpu_set_accel_fsr(16); // set fsr for accel
    mpu_set_gyro_fsr(2000); // set fsr for gyro

    mimsyDmpBegin();

    // clear local variables
    memset(&uinject_vars,0,sizeof(uinject_vars_t));

    // register at UDP stack
    uinject_vars.desc.port              = WKP_UDP_INJECT;
    uinject_vars.desc.callbackReceive   = &uinject_receive;
    uinject_vars.desc.callbackSendDone  = &uinject_sendDone;
    openudp_register(&uinject_vars.desc);

    uinject_vars.period = UINJECT_PERIOD_MS;
    // start periodic timer
    uinject_vars.timerId = opentimers_create();
    opentimers_scheduleIn(
        uinject_vars.timerId,
        UINJECT_PERIOD_MS,
        TIME_MS,
        TIMER_PERIODIC,
        uinject_timer_cb
    );
}

void uinject_sendDone(OpenQueueEntry_t* msg, owerror_t error) {
   openqueue_freePacketBuffer(msg);
}

void uinject_receive(OpenQueueEntry_t* pkt) {

   openqueue_freePacketBuffer(pkt);

   openserial_printError(
      COMPONENT_UINJECT,
      ERR_RCVD_ECHO_REPLY,
      (errorparameter_t)0,
      (errorparameter_t)0
   );
}

//=========================== private =========================================

/**
\note timer fired, but we don't want to execute task in ISR mode instead, push
   task to scheduler with CoAP priority, and let scheduler take care of it.
*/
void uinject_timer_cb(opentimers_id_t id){

   scheduler_push_task(uinject_task_cb,TASKPRIO_COAP);
}

void uinject_task_cb() {

    i2c_init();
    //mimsyIMUInit();

    //mpu_set_sensors(INV_XYZ_GYRO|INV_XYZ_ACCEL); // turn on sensors
    //mpu_set_accel_fsr(16); // set fsr for accel
    //mpu_set_gyro_fsr(2000); // set fsr for gyro

    //mimsyDmpBegin();


    OpenQueueEntry_t*    pkt;
    uint8_t              asnArray[5];

    // don't run if not synch
    if (ieee154e_isSynch() == FALSE) return;

    // don't run on dagroot
    if (idmanager_getIsDAGroot()) {
      opentimers_destroy(uinject_vars.timerId);
      return;
    }

    // read imu fifo
    short gyro[3] = {0,0,0};
    short accel[3] = {0,0,0};
    long quat[4] = {0,0,0,0};
    short sensors = INV_XYZ_GYRO | INV_WXYZ_QUAT | INV_XYZ_ACCEL;
    short more;
    unsigned long timestamp;

    while(dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more) != 0) {}

    union {
      float flt;
      unsigned char bytes[4];
    } yaw;

    union {
      float flt;
      unsigned char bytes[4];
    } pitch;

    union {
      float flt;
      unsigned char bytes[4];
    } roll;

    quat2euler(quat, &(yaw.flt), &(pitch.flt), &(roll.flt));

    // if you get here, send a packet

    // get a free packet buffer
    pkt = openqueue_getFreePacketBuffer(COMPONENT_UINJECT);
    if (pkt==NULL) {
      openserial_printError(
         COMPONENT_UINJECT,
         ERR_NO_FREE_PACKET_BUFFER,
         (errorparameter_t)0,
         (errorparameter_t)0
      );
      return;
    }

    pkt->owner                         = COMPONENT_UINJECT;
    pkt->creator                       = COMPONENT_UINJECT;
    pkt->l4_protocol                   = IANA_UDP;
    pkt->l4_destination_port           = WKP_UDP_INJECT;
    pkt->l4_sourcePortORicmpv6Type     = WKP_UDP_INJECT;
    pkt->l3_destinationAdd.type        = ADDR_128B;
    memcpy(&pkt->l3_destinationAdd.addr_128b[0],uinject_dst_addr,16);

    // add payload
    packetfunctions_reserveHeaderSize(pkt,sizeof(uinject_payload)-1);
    memcpy(&pkt->payload[0],uinject_payload,sizeof(uinject_payload)-1);

 

    // add payload
  	packetfunctions_reserveHeaderSize(pkt,6*sizeof(uint16_t)+3*sizeof(float));
  	pkt->payload[1] = (uint8_t)((accel[0] & 0xff00)>>8);
  	pkt->payload[0] = (uint8_t)(accel[0] & 0x00ff);
  	pkt->payload[3] = (uint8_t)((accel[1] & 0xff00)>>8);
  	pkt->payload[2] = (uint8_t)(accel[1] & 0x00ff);
  	pkt->payload[5] = (uint8_t)((accel[2] & 0xff00)>>8);
  	pkt->payload[4] = (uint8_t)(accel[2] & 0x00ff);
  	pkt->payload[7] = (uint8_t)((gyro[0] & 0xff00)>>8);
  	pkt->payload[6] = (uint8_t)(gyro[0] & 0x00ff);
  	pkt->payload[9] = (uint8_t)((gyro[1] & 0xff00)>>8);
  	pkt->payload[8] = (uint8_t)(gyro[1] & 0x00ff);
  	pkt->payload[11] = (uint8_t)((gyro[2] & 0xff00)>>8);
    pkt->payload[10] = (uint8_t)(gyro[2] & 0x00ff);
  	//*************** euler angles ****************************
  	pkt->payload[15] = roll.bytes[3];
  	pkt->payload[14] = roll.bytes[2];
  	pkt->payload[13] = roll.bytes[1];
  	pkt->payload[12] = roll.bytes[0];
  	pkt->payload[19] = pitch.bytes[3];
  	pkt->payload[18] = pitch.bytes[2];
  	pkt->payload[17] = pitch.bytes[1];
  	pkt->payload[16] = pitch.bytes[0];
  	pkt->payload[23] = yaw.bytes[3];
  	pkt->payload[22] = yaw.bytes[2];
  	pkt->payload[21] = yaw.bytes[1];
  	pkt->payload[20] = yaw.bytes[0];

    packetfunctions_reserveHeaderSize(pkt,sizeof(uint16_t));
    pkt->payload[1] = (uint8_t)(idmanager_getMyID(ADDR_64B)->addr_64b[6]);
    pkt->payload[0] = (uint8_t)(idmanager_getMyID(ADDR_64B)->addr_64b[7]);

    uinject_vars.counter++;

    packetfunctions_reserveHeaderSize(pkt,sizeof(asn_t));
    ieee154e_getAsn(asnArray);
    pkt->payload[0] = asnArray[0];
    pkt->payload[1] = asnArray[1];
    pkt->payload[2] = asnArray[2];
    pkt->payload[3] = asnArray[3];
    pkt->payload[4] = asnArray[4];

    if ((openudp_send(pkt))==E_FAIL) {
      openqueue_freePacketBuffer(pkt);
    }
}

float mysqrt(float square)
{
    float root=square/3;
    int i;
    if (square <= 0) return 0;
    for (i=0; i<32; i++)
        root = (root + square / root) / 2;
    return root;
}

float mypow (float num, int pow) {
	int i;
	float result = 1;
	for (i = pow; i > 0; i--) {
		result *= num;
	}
	return result;
}

void quat2euler(long * quat, float * yaw, float * pitch, float * roll) {
   float fquats[4];
   //conversion to float
   fquats[0]=(float)quat[0]/(float)0x40000000;
   fquats[1]=(float)quat[1]/(float)0x40000000;
   fquats[2]=(float)quat[2]/(float)0x40000000;
   fquats[3]=(float)quat[3]/(float)0x40000000;
   
   //inv_q_norm4(fquats);
   float mag;
    mag = mysqrt(fquats[0] * fquats[0] + fquats[1] * fquats[1] + fquats[2] * fquats[2] + fquats[3] * fquats[3]);
    if (mag) {
        fquats[0] /= mag;
        fquats[1] /= mag;
        fquats[2] /= mag;
        fquats[3] /= mag;
    } else {
        fquats[0] = 1.f;
        fquats[1] = 0.f;
        fquats[2] = 0.f;
        fquats[3] = 0.f;
    }

   float myarg1;
   float myarg2;
   myarg1 = 2 * (fquats[0] * fquats[2] - fquats[3] * fquats[1]);
   *pitch = myarg1 + (mypow(myarg1,3) / 6) + (3 * mypow(myarg1,5) / 40.0) + (5*mypow(myarg1,7) / 112.0) + (35*mypow(myarg1,9) / 1152);

   myarg1 =  2 * (fquats[0] * fquats[3] + fquats[1] * fquats[2]);
   myarg2 = 1 - 2 * (fquats[2]*fquats[2] + fquats[3] * fquats[3]);
   myarg1 = myarg1/myarg2;
   if (myarg1 <= 1 && myarg1 >= -1) {
   *yaw = myarg1 - mypow(myarg1,3)/3.0 + mypow(myarg1,5)/5.0 - mypow(myarg1,7)/7.0 + mypow(myarg1,9)/9.0;
	}
	else if (myarg1 > 1) {
		myarg1 = 1.0/myarg1;
		*yaw = 1.570796326794897 - (myarg1 - mypow(myarg1,3)/3.0 + mypow(myarg1,5)/5.0 - mypow(myarg1,7)/7.0 + mypow(myarg1,9)/9.0);
	}
	else {
		myarg1 = -1.0/myarg1;
		*yaw = -1.0*(1.570796326794897 - (myarg1 - mypow(myarg1,3)/3.0 + mypow(myarg1,5)/5.0 - mypow(myarg1,7)/7.0 + mypow(myarg1,9)/9.0));
	}


   myarg1 =  2 * (fquats[0] * fquats[1] + fquats[2] * fquats[3]);
   myarg2 = 1 - 2 * (fquats[1] * fquats[1] + fquats[2] * fquats[2]);
   myarg1 = myarg1/myarg2;
	if (myarg1 <= 1 && myarg1 >= -1) {
   *roll = myarg1 - mypow(myarg1,3)/3.0 + mypow(myarg1,5)/5.0 - mypow(myarg1,7)/7.0 + mypow(myarg1,9)/9.0;
	}
   else if (myarg1 > 1) {
   	myarg1 = 1.0/myarg1;
   	*roll = 1.570796326794897 - (myarg1 - mypow(myarg1,3)/3.0 + mypow(myarg1,5)/5.0 - mypow(myarg1,7)/7.0 + mypow(myarg1,9)/9.0);
   }
   else {
   	myarg1 = -1.0/myarg1;
   	*roll = -1.0*(1.570796326794897 - (myarg1 - mypow(myarg1,3)/3.0 + mypow(myarg1,5)/5.0 - mypow(myarg1,7)/7.0 + mypow(myarg1,9)/9.0));
   }

   //*pitch = asinf(2 * (fquats[0] * fquats[2] - fquats[3] * fquats[1])); //computes sin of pitch
   //gyro yaw
   //*yaw = atan2f(2 * (fquats[0] * fquats[3] + fquats[1] * fquats[2]), 1 - 2 * (fquats[2]*fquats[2] + fquats[3] * fquats[3]));
   //roll control
   //*roll= atan2f(2 * (fquats[0] * fquats[1] + fquats[2] * fquats[3]), 1 - 2 * (fquats[1] * fquats[1] + fquats[2] * fquats[2]));
   
}
