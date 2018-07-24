#include "kl_lib.h"
#include "board.h"
#include "led.h"
#include "vibro.h"
#include "Sequences.h"
#include "radio_lvl1.h"
#include "kl_i2c.h"
#include "MsgQ.h"
#include "main.h"
#include "acc_mma8452.h"
#include "kl_adc.h"

#if 1 // ======================== Variables and defines ========================
// Forever
EvtMsgQ_t<EvtMsg_t, MAIN_EVT_Q_LEN> EvtQMain;
static const UartParams_t CmdUartParams(115200, CMD_UART_PARAMS);
CmdUart_t Uart{&CmdUartParams};
static void ITask();
static void OnCmd(Shell_t *PShell);

#define EE_ADDR_DEVICE_ID       0

uint8_t ID;
static const PinInputSetup_t DipSwPin[DIP_SW_CNT] = { DIP_SW6, DIP_SW5, DIP_SW4, DIP_SW3, DIP_SW2, DIP_SW1 };
static uint8_t GetDipSwitch();
static uint8_t ISetID(int32_t NewID);
void ReadIDfromEE();

void ReadAndSetupMode();
void EnterIdle();
void CheckRxTable();

LedSmooth_t Led {LED_CTRL_PIN, 1000}; // 2500Hz PWM to allow ST1CC40 to handle it
Vibro_t Vibro {VIBRO_SETUP};

uint8_t SignalTx = SIGN_SILMARIL; // Always
bool IsIdle = true;
bool IsVibrating = true;

Acc_t Acc(&i2c1);
PinOutput_t AccPwr(ACC_PWR_PIN);

// ==== Timers ====
static TmrKL_t TmrEverySecond {MS2ST(1000), evtIdEverySecond, tktPeriodic};
static TmrKL_t TmrNoMovement  {MS2ST(4500), evtIdNoMove, tktOneShot};
static TmrKL_t TmrRxTableCheck {MS2ST(3600), evtIdCheckRxTable, tktPeriodic};
static int32_t TimeS;
#endif

int main(void) {
    // ==== Init Vcore & clock system ====
    SetupVCore(vcore1V5);
    Clk.SetMSI4MHz();
    Clk.EnableHSI();    // Required foe ADC
    Clk.UpdateFreqValues();

    // === Init OS ===
    halInit();
    chSysInit();
    EvtQMain.Init();

    // ==== Init hardware ====
    Uart.Init();
    Uart.StartRx();
    ReadIDfromEE();
    Printf("\r%S %S ID=%u\r", APP_NAME, XSTRINGIFY(BUILD_TIME), ID);
    Clk.PrintFreqs();

    Led.Init();
    Vibro.Init();
    Vibro.StartOrRestart(vsqBrr);
    Vibro.SetupSeqEndEvt(evtIdVibroEnd);

    AccPwr.Init();
    AccPwr.SetHi();
    chThdSleepMilliseconds(18);
    i2c1.Init();
//    i2c1.ScanBus();
    Acc.Init();

    TmrEverySecond.StartOrRestart();
    TmrRxTableCheck.StartOrRestart();

    EnterIdle();

    Radio.Init();

    // ==== Adc ====
    PinSetupAnalog(BAT_MEAS_PIN);
//    CalibrationCounter
    Adc.Init();
    Adc.EnableVRef();

    // Main cycle
    ITask();
}

__noreturn
void ITask() {
    while(true) {
        EvtMsg_t Msg = EvtQMain.Fetch(TIME_INFINITE);
        switch(Msg.ID) {
            case evtIdEverySecond:
                TimeS++;
                ReadAndSetupMode();
                break;

            case evtIdCheckRxTable: CheckRxTable(); break;

            case evtIdNoMove: if(IsIdle) Led.StartOrContinue(lsqDim); break;

            case evtIdAcc:
//                Printf("Acc\r");
                if(IsIdle and !IsVibrating) {
                    Led.StartOrRestart(lsqTop);
                    TmrNoMovement.StartOrRestart();
                }
                break;

            case evtIdAdcRslt:
//                Printf("Ubat=%u\r", Msg.Value*2);
                if(Msg.Value*2 < 3600) {
                    IsVibrating = true; // Ignore Acc when vibrating
                    Vibro.StartOrContinue(vsqDischarged);
                }
                break;

            case evtIdVibroEnd:
//                Printf("VibroEnd\r");
                IsVibrating = false;
                break;

            case evtIdShellCmd:
                OnCmd((Shell_t*)Msg.Ptr);
                ((Shell_t*)Msg.Ptr)->SignalCmdProcessed();
                break;

            default: Printf("Unhandled Msg %u\r", Msg.ID); break;
        } // Switch
    } // while true
} // ITask()

void CheckRxTable() {
    uint32_t Cnt = Radio.RxTable.GetCount();
    uint8_t OatherCnt = 0;
    if(Cnt > 0) {
        // Analyze RxTable
        for(uint32_t i=0; i<Cnt; i++) {
            if(Radio.RxTable.Buf[i].Signal & SIGN_OATH) {
                OatherCnt++;
                if(OatherCnt == 3) break; // Stop if
            }
        }
        Radio.RxTable.Clear();
    } // Cnt > 0
    // Process what counted
    if(OatherCnt == 0) {
        if(!IsIdle) {
            EnterIdle();
            IsIdle = true;
        }
    }
    else {
        IsIdle = false;
        TmrNoMovement.Stop();
        Led.StartOrContinue(lsqOathIsNear);
    }
}

void EnterIdle() {
    TmrNoMovement.StartOrRestart();
    Led.StartOrRestart(lsqTop);
}

__unused
static const uint8_t PwrTable[12] = {
        CC_PwrMinus30dBm, // 0
        CC_PwrMinus27dBm, // 1
        CC_PwrMinus25dBm, // 2
        CC_PwrMinus20dBm, // 3
        CC_PwrMinus15dBm, // 4
        CC_PwrMinus10dBm, // 5
        CC_PwrMinus6dBm,  // 6
        CC_Pwr0dBm,       // 7
        CC_PwrPlus5dBm,   // 8
        CC_PwrPlus7dBm,   // 9
        CC_PwrPlus10dBm,  // 10
        CC_PwrPlus12dBm   // 11
};

__unused
void ReadAndSetupMode() {
    static uint8_t OldDipSettings = 0xFF;
    uint8_t b = GetDipSwitch();
    if(b == OldDipSettings) return;
    // ==== Something has changed ====
    Printf("Dip: 0x%02X\r", b);
    OldDipSettings = b;
    RMsg_t msg;
    // Select TX pwr
    msg.Cmd = rmsgSetPwr;
    b &= 0b1111; // Remove high bits
    msg.Value = (b > 11)? CC_PwrPlus12dBm : PwrTable[b];
    Printf("Pwr=%u\r", b);
    Radio.RMsgQ.SendNowOrExit(msg);
}


#if 1 // ========================= Command processing ==========================
void OnCmd(Shell_t *PShell) {
	Cmd_t *PCmd = &PShell->Cmd;
    // Handle command
    if(PCmd->NameIs("Ping")) {
        PShell->Ack(retvOk);
    }
    else if(PCmd->NameIs("Version")) PShell->Printf("%S %S\r", APP_NAME, XSTRINGIFY(BUILD_TIME));

    else if(PCmd->NameIs("GetID")) PShell->Reply("ID", ID);

    else if(PCmd->NameIs("SetID")) {
        if(PCmd->GetNext<uint8_t>(&ID) != retvOk) { PShell->Ack(retvCmdError); return; }
        uint8_t r = ISetID(ID);
//        RMsg_t msg;
//        msg.Cmd = R_MSG_SET_CHNL;
//        msg.Value = ID2RCHNL(ID);
//        Radio.RMsgQ.SendNowOrExit(msg);
        PShell->Ack(r);
    }

    else PShell->Ack(retvCmdUnknown);
}
#endif

#if 1 // =========================== ID management =============================
void ReadIDfromEE() {
    ID = EE::Read32(EE_ADDR_DEVICE_ID);  // Read device ID
    if(ID < ID_MIN or ID > ID_MAX) {
        Printf("\rUsing default ID\r");
        ID = 0;
    }
}

uint8_t ISetID(int32_t NewID) {
    if(NewID < ID_MIN or NewID > ID_MAX) return retvFail;
    uint8_t rslt = EE::Write32(EE_ADDR_DEVICE_ID, NewID);
    if(rslt == retvOk) {
        ID = NewID;
        Printf("New ID: %u\r", ID);
        return retvOk;
    }
    else {
        Printf("EE error: %u\r", rslt);
        return retvFail;
    }
}
#endif

// ====== DIP switch ======
uint8_t GetDipSwitch() {
    uint8_t Rslt = 0;
    for(int i=0; i<DIP_SW_CNT; i++) PinSetupInput(DipSwPin[i].PGpio, DipSwPin[i].Pin, DipSwPin[i].PullUpDown);
    for(int i=0; i<DIP_SW_CNT; i++) {
        if(!PinIsHi(DipSwPin[i].PGpio, DipSwPin[i].Pin)) Rslt |= (1 << i);
        PinSetupAnalog(DipSwPin[i].PGpio, DipSwPin[i].Pin);
    }
    return Rslt;
}
