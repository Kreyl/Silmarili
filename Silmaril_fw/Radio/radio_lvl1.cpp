/*
 * radio_lvl1.cpp
 *
 *  Created on: Nov 17, 2013
 *      Author: kreyl
 */

#include "radio_lvl1.h"
#include "cc1101.h"
#include "uart.h"
#include "main.h"

cc1101_t CC(CC_Setup0);

#define DBG_PINS

#ifdef DBG_PINS
#define DBG_GPIO1   GPIOA
#define DBG_PIN1    0
#define DBG1_SET()  PinSetHi(DBG_GPIO1, DBG_PIN1)
#define DBG1_CLR()  PinSetLo(DBG_GPIO1, DBG_PIN1)
#define DBG_GPIO2   GPIOB
#define DBG_PIN2    9
#define DBG2_SET()  PinSetHi(DBG_GPIO2, DBG_PIN2)
#define DBG2_CLR()  PinSetLo(DBG_GPIO2, DBG_PIN2)
#else
#define DBG1_SET()
#define DBG1_CLR()
#define DBG2_SET()
#define DBG2_CLR()
#endif

rLevel1_t Radio;
void TmrTimeslotCallback(void *p);
//static volatile enum CCState_t {ccstIdle, ccstRx, ccstTx} CCState = ccstIdle;
static bool InsideTx = false;
extern uint8_t SignalTx;

static Timer_t SyncTmr(TIM9);
#define TICS_TOTAL  (TIMESLOT_DURATION_TICS * TIMESLOT_CNT * RCYCLE_CNT)

static class RadioTime_t {
private:
    uint16_t TimeSrcTimeout = 0;
public:
    uint16_t TimeSrcID;
    void Adjust(rPkt_t &Pkt) {
        if((Pkt.TimeSrcID < TimeSrcID) or ((Pkt.TimeSrcID == TimeSrcID) and (Pkt.ID < ID))) {
            chSysLock();
            uint32_t AlienTime = Pkt.Time + 27;
            PrintfI("Src: %u/%u;  %u; %u; %u\r", Pkt.TimeSrcID, TimeSrcID, Pkt.Time, AlienTime, SyncTmr.GetCounter());
            SyncTmr.SetCounter(AlienTime);
            TimeSrcID = Pkt.ID;
            TimeSrcTimeout = SCYCLES_TO_KEEP_TIMESRC; // Reset Time Src Timeout
            chSysUnlock();
        }
    }

    void OnNewSupercycleI() {
        if(ID == 0) {
            InsideTx = true;
            Radio.RMsgQ.SendNowOrExitI(RMsg_t(rmsgTimeToTx)); // Enter TX
            uint32_t NextTime = TIMESLOT_DURATION_TICS;
            SyncTmr.SetCCR1(NextTime); // Will fire at end of TX timeslot
        }
        else {
            SyncTmr.SetCCR1(TIMESLOT_DURATION_TICS * ID); // Will fire at start of TX timeslot
        }
        // Process TimeSrc
        if(TimeSrcTimeout > 0) TimeSrcTimeout--;
        else TimeSrcID = ID;
    }

    void OnCompareI(uint32_t CurrentTick) {
        if(InsideTx) { // End of TX slot
            InsideTx = false;
            uint32_t NextTime = CurrentTick + (TIMESLOT_CNT - 1) * TIMESLOT_DURATION_TICS;
            // Do not setup new TX slot if too close to end of supercycle
            if(NextTime < (TICS_TOTAL - TIMESLOT_DURATION_TICS)) SyncTmr.SetCCR1(NextTime); // Will fire at start of TX timeslot
            if(CurrentTick < (TIMESLOT_CNT * TIMESLOT_DURATION_TICS)) { // Zero cycle
                Radio.RMsgQ.SendNowOrExitI(RMsg_t(rmsgTimeToRx)); // Enter RX
            }
            else { // Non-zero cycle
                Radio.RMsgQ.SendNowOrExitI(RMsg_t(rmsgTimeToSleep));
            }
        }
        else { // Start of TX slot
            InsideTx = true;
            uint32_t NextTime = CurrentTick + TIMESLOT_DURATION_TICS;
            SyncTmr.SetCCR1(NextTime); // Will fire at end of TX timeslot
            Radio.RMsgQ.SendNowOrExitI(RMsg_t(rmsgTimeToTx)); // Enter TX
        }
    }
} RadioTime;

// SyncTimer IRQ
extern "C"
void VectorA4() {
    CH_IRQ_PROLOGUE();
    chSysLockFromISR();
    if(SyncTmr.IsUpdateIrqFired()) {
        SyncTmr.ClearUpdateIrqPendingBit();
        RadioTime.OnNewSupercycleI();
    }
    if(SyncTmr.IsCompare1IrqFired()) {
        SyncTmr.ClearCompare1IrqPendingBit();
        RadioTime.OnCompareI(SyncTmr.GetCounter());
    }
    chSysUnlockFromISR();
    CH_IRQ_EPILOGUE();
}

void RxCallback() {
    Radio.RMsgQ.SendNowOrExitI(RMsg_t(rmsgPktRx));
}


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
        RMsg_t msg = RMsgQ.Fetch(TIME_INFINITE);
        switch(msg.Cmd) {
            case rmsgTimeToTx:
//                Printf("Tttx\r");
                DBG2_CLR();
                PktTx.ID = ID;
                PktTx.Time = SyncTmr.GetCounter();
                PktTx.TimeSrcID = RadioTime.TimeSrcID;
                PktTx.Signal = SignalTx;
//                PktTx.Print();
                DBG1_SET();
                CC.Recalibrate();
                CC.Transmit(&PktTx, RPKT_LEN);
                DBG1_CLR();
                break;

            case rmsgTimeToRx:
//                Printf("Ttrx\r");
                DBG2_SET();
                CC.ReceiveAsync(RxCallback);
                break;

            case rmsgTimeToSleep:
                DBG2_CLR();
                CC.EnterIdle();
                break;

            case rmsgPktRx:
                DBG2_CLR();
                if(CC.ReadFIFO(&PktRx, &Rssi, RPKT_LEN) == retvOk) {  // if pkt successfully received
//                    Printf("%d; ", Rssi);
//                    PktRx.Print();
//                    LocalTable.AddOrReplaceExistingPkt(PktRx);
                    RadioTime.Adjust(PktRx);
                    RxTable.AddOrReplaceExistingPkt(PktRx);
                }
                CC.ReceiveAsync(RxCallback);
                DBG2_SET();
                break;

            case rmsgSetPwr: CC.SetTxPower(msg.Value); break;

            case rmsgSetChnl:
//                CC.SetChannel(msg.Value);
                break;
        } // switch
    } // while
}

#endif // task

#if 1 // ============================
uint8_t rLevel1_t::Init() {
#ifdef DBG_PINS
    PinSetupOut(DBG_GPIO1, DBG_PIN1, omPushPull);
    PinSetupOut(DBG_GPIO2, DBG_PIN2, omPushPull);
#endif

    RMsgQ.Init();
    if(CC.Init() == retvOk) {
        CC.SetTxPower(CC_PwrMinus30dBm);
        CC.SetPktSize(RPKT_LEN);
        CC.SetChannel(RCHNL);

        SyncTmr.Init();
        SyncTmr.SetupPrescaler(10000); // 10kHz => 10 tics in 1 ms
        // Total amount of timeslots in supercycle
        SyncTmr.SetTopValue(TICS_TOTAL);
        // Irq
        SyncTmr.EnableIrqOnUpdate();
        SyncTmr.EnableIrqOnCompare1();
        SyncTmr.EnableIrq(TIM9_IRQn, IRQ_PRIO_MEDIUM);
        SyncTmr.GenerateUpdateEvt();

        // Thread
        chThdCreateStatic(warLvl1Thread, sizeof(warLvl1Thread), HIGHPRIO, (tfunc_t)rLvl1Thread, NULL);
        SyncTmr.Enable();
        return retvOk;
    }
    else return retvFail;
}
#endif
