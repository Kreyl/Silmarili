#include "kl_lib.h"
#include "board.h"
#include "led.h"
#include "Sequences.h"
#include "kl_i2c.h"
#include "MsgQ.h"
#include "main.h"
#include "acc_mma8452.h"

#if 1 // ======================== Variables and defines ========================
// Forever
EvtMsgQ_t<EvtMsg_t, MAIN_EVT_Q_LEN> EvtQMain;
static const UartParams_t CmdUartParams(115200, CMD_UART_PARAMS);
CmdUart_t Uart{&CmdUartParams};
static void ITask();
static void OnCmd(Shell_t *PShell);

void EnterIdle();

LedSmooth_t Led {LED_CTRL_PIN, 1000}; // 2500Hz PWM to allow ST1CC40 to handle it

Acc_t Acc(&i2c1);
PinOutput_t AccPwr(ACC_PWR_PIN);

static TmrKL_t TmrNoMovement  {MS2ST(4500), evtIdNoMove, tktOneShot};
#endif

int main(void) {
    // ==== Init Vcore & clock system ====
    SetupVCore(vcore1V5);
//    Clk.SetMSI4MHz();
    Clk.UpdateFreqValues();

    // === Init OS ===
    halInit();
    chSysInit();
    EvtQMain.Init();

    // ==== Init hardware ====
    Uart.Init();
//    Uart.StartRx();
    Printf("\r%S %S\r", APP_NAME, XSTRINGIFY(BUILD_TIME));
    Clk.PrintFreqs();

    Led.Init();

    AccPwr.Init();
    AccPwr.SetHi();
    chThdSleepMilliseconds(18);
    i2c1.Init();
//    i2c1.ScanBus();
    Acc.Init();

    EnterIdle();

    // Main cycle
    ITask();
}

__noreturn
void ITask() {
    while(true) {
        EvtMsg_t Msg = EvtQMain.Fetch(TIME_INFINITE);
        switch(Msg.ID) {
            case evtIdNoMove: Led.StartOrContinue(lsqDim); break;

            case evtIdAcc:
                Led.StartOrRestart(lsqTop);
                TmrNoMovement.StartOrRestart();
                break;

            case evtIdShellCmd:
                OnCmd((Shell_t*)Msg.Ptr);
                ((Shell_t*)Msg.Ptr)->SignalCmdProcessed();
                break;

            default: Printf("Unhandled Msg %u\r", Msg.ID); break;
        } // Switch
    } // while true
} // ITask()

void EnterIdle() {
    TmrNoMovement.StartOrRestart();
    Led.StartOrRestart(lsqTop);
}

#if 1 // ========================= Command processing ==========================
void OnCmd(Shell_t *PShell) {
	Cmd_t *PCmd = &PShell->Cmd;
    // Handle command
    if(PCmd->NameIs("Ping")) {
        PShell->Ack(retvOk);
    }
    else if(PCmd->NameIs("Version")) PShell->Printf("%S %S\r", APP_NAME, XSTRINGIFY(BUILD_TIME));

    else PShell->Ack(retvCmdUnknown);
}
#endif
