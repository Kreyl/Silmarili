#include "board.h"
#include "led.h"
#include "vibro.h"
#include "Sequences.h"
#include "radio_lvl1.h"
#include "kl_i2c.h"
#include "kl_lib.h"
#include "kl_buf.h"
#include "MsgQ.h"
#include "acc_mma8452.h"
#include "adcL151.h"

#include "Config.h"

#include <vector>

#if 1 // ======================== Variables and defines ========================
// Forever
EvtMsgQ_t<EvtMsg_t, MAIN_EVT_Q_LEN> EvtQMain;
static const UartParams_t CmdUartParams(115200, CMD_UART_PARAMS);
CmdUart_t Uart{&CmdUartParams};
static void ITask();
static void OnCmd(Shell_t *PShell);

static void ReadAndSetupMode();

// EEAddresses
#define EE_ADDR_DEVICE_ID       0

static const PinInputSetup_t DipSwPin[DIP_SW_CNT] = { DIP_SW6, DIP_SW5, DIP_SW4, DIP_SW3, DIP_SW2, DIP_SW1 };
static uint8_t GetDipSwitch();

// ==== Periphery ====
Vibro_t Vibro {VIBRO_SETUP};
LedSmooth_t Led {LED_CTRL_PIN, 1000}; // 2500Hz PWM to allow ST1CC40 to handle it

bool IsIdle = true;
bool IsVibrating = true;

Acc_t Acc(&i2c1);
PinOutput_t AccPwr(ACC_PWR_PIN);

// ==== Timers ====
static TmrKL_t TmrEverySecond {TIME_MS2I(1000), evtIdEverySecond, tktPeriodic};
static TmrKL_t TmrNoMovement  {TIME_MS2I(4500), evtIdNoMove, tktOneShot};
static TmrKL_t TmrRxTableCheck {TIME_MS2I(3600), evtIdCheckRxTable, tktPeriodic};
static uint32_t TimeS;
#endif

#if 1 // ========================== Logic ======================================
// === Types ===
#define TYPE_OBSERVER           0
#define TYPE_NOTHING            0 // What to show
#define TYPE_DARKSIDE           1
#define TYPE_LIGHTSIDE          2
#define TYPE_BOTH               3 // Allow to tx just in case

#define TYPE_CNT                4

void EnterIdle() {
    TmrNoMovement.StartOrRestart();
    Led.StartOrRestart(lsqDim);
}

void CheckRxTable() {
    // Analyze table: get count of every type near
    uint32_t TypesCnt[TYPE_CNT] = {0};
    RxTable_t *Tbl = Radio.GetRxTable();
    Tbl->ProcessCountingDistinctTypes(TypesCnt, TYPE_CNT);
    uint32_t Cnt = TypesCnt[TYPE_DARKSIDE] + TypesCnt[TYPE_LIGHTSIDE] + TypesCnt[TYPE_BOTH];
    // Fade if nobody around
    if(Cnt == 0) {
        if(!IsIdle) {
            EnterIdle();
            IsIdle = true;
        }
    }
    else {
        IsIdle = false;
        TmrNoMovement.Stop();
        Led.StartOrContinue(lsqMagicIsNear);
    }
}
#endif

int main(void) {
    // ==== Init Vcore & clock system ====
    SetupVCore(vcore1V2);
    Clk.SetMSI4MHz();
    Clk.EnableHSI();    // Required foe ADC
    Clk.UpdateFreqValues();

    // === Init OS ===
    halInit();
    chSysInit();
    EvtQMain.Init();

    // ==== Init hardware ====
    Uart.Init();
    Printf("\r%S %S\r", APP_NAME, XSTRINGIFY(BUILD_TIME));
    Clk.PrintFreqs();
    Random::Seed(GetUniqID3());   // Init random algorythm with uniq ID

    Led.Init();
    Vibro.Init();
    Vibro.Init();
    Vibro.StartOrRestart(vsqBrr);
    Vibro.SetupSeqEndEvt(evtIdVibroSeqDone);

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
//                ReadAndSetupMode();
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

            case evtIdVibroSeqDone:
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

__unused
void ReadAndSetupMode() {
    static uint8_t OldDipSettings = 0xFF;
    uint8_t b = GetDipSwitch();
    if(b == OldDipSettings) return;
    // ==== Something has changed ====
    Printf("Dip: 0x%02X\r", b);
    OldDipSettings = b;
//    RMsg_t msg;
    // Select TX pwr
//    msg.Cmd = rmsgSetPwr;
//    b &= 0b1111; // Remove high bits
//    msg.Value = (b > 11)? CC_PwrPlus12dBm : PwrTable[b];
//    Printf("Pwr=%u\r", b);
//    Radio.RMsgQ.SendNowOrExit(msg);
}

#if 1 // ================= Command processing ====================
void OnCmd(Shell_t *PShell) {
	Cmd_t *PCmd = &PShell->Cmd;
    // Handle command
    if(PCmd->NameIs("Ping")) PShell->Ok();
    else if(PCmd->NameIs("Version")) PShell->Print("%S %S\r", APP_NAME, XSTRINGIFY(BUILD_TIME));

    else PShell->CmdUnknown();
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
