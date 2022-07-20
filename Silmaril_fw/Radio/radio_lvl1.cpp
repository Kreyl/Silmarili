/*
 * radio_lvl1.cpp
 *
 *  Created on: Nov 17, 2013
 *      Author: kreyl
 */

#include "radio_lvl1.h"
#include "cc1101.h"

cc1101_t CC(CC_Setup0);

//#define DBG_PINS

#ifdef DBG_PINS
#define DBG_GPIO1   GPIOB
#define DBG_PIN1    10
#define DBG1_SET()  PinSetHi(DBG_GPIO1, DBG_PIN1)
#define DBG1_CLR()  PinSetLo(DBG_GPIO1, DBG_PIN1)
#define DBG_GPIO2   GPIOB
#define DBG_PIN2    11
#define DBG2_SET()  PinSetHi(DBG_GPIO2, DBG_PIN2)
#define DBG2_CLR()  PinSetLo(DBG_GPIO2, DBG_PIN2)
#else
#define DBG1_SET()
#define DBG1_CLR()
#endif

rLevel1_t Radio;
int8_t Rssi;
static rPkt_t PktRx;
static bool IsSleeping = false;

#if 1 // ================================ Task =================================
static THD_WORKING_AREA(warLvl1Thread, 256);
__noreturn
static void rLvl1Thread(void *arg) {
    chRegSetThreadName("rLvl1");
    Radio.ITask();
}

__noreturn
void rLevel1_t::ITask() {
    while(true) {
        CC.Recalibrate();
        systime_t Start = chVTGetSystemTimeX();
        while(chVTTimeElapsedSinceX(Start) < TIME_MS2I(RX_DURATION_MS) and Enabled) {
            if(CC.Receive(RX_DURATION_MS, (uint8_t*)&PktRx, RPKT_LEN, &Rssi) == retvOk) {
//                Printf("ID: %u; Type: %u; Rssi: %d\r", PktRx.ID, PktRx.Type, Rssi);
                if(PktRx.Salt == RPKT_SALT) {
                    chSysLock();
                    Radio.AddPktToRxTableI(&PktRx);
                    chSysUnlock();
                }
            }
        }
        CC.PowerOff();
        IsSleeping = true;
        chThdSleepMilliseconds(SLEEP_DURATION_MS);
        IsSleeping = false;
    } // while true
}

void rLevel1_t::Stop() {
    Enabled = false;
    while(!IsSleeping) chThdSleepMilliseconds(7);
}
#endif // task

#if 1 // ============================
uint8_t rLevel1_t::Init() {
#ifdef DBG_PINS
    PinSetupOut(DBG_GPIO1, DBG_PIN1, omPushPull);
    PinSetupOut(DBG_GPIO2, DBG_PIN2, omPushPull);
#endif

    if(CC.Init() == retvOk) {
        CC.SetPktSize(RPKT_LEN);
        CC.SetChannel(RCHNL_EACH_OTH);
        CC.SetTxPower(CC_Pwr0dBm); // dummy
        CC.SetBitrate(CCBitrate500k);
//        CC.EnterPwrDown();

        chThdCreateStatic(warLvl1Thread, sizeof(warLvl1Thread), HIGHPRIO, (tfunc_t)rLvl1Thread, NULL);
        return retvOk;
    }
    else return retvFail;
}
#endif
