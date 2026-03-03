#include "main.h"
#include "tx_api.h"

extern TX_THREAD ControlThread;
extern uint8_t ControlThreadStack[4096];
extern void ControlThreadFun(ULONG initial_input);

extern TX_THREAD OperateThread;
extern uint8_t OperateThreadStack[2048];
extern void OperateThreadFun(ULONG initial_input);

#define TX_NAME(s) const_cast<CHAR*>(s)
extern "C" void TaskBooster(void)
{
    tx_thread_create(&ControlThread, TX_NAME("ControlThread"), ControlThreadFun, 0x1234, ControlThreadStack, sizeof(ControlThreadStack), 1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);
    tx_thread_create(&OperateThread, TX_NAME("OperateThread"), OperateThreadFun, 0x1234, OperateThreadStack, sizeof(OperateThreadStack), 2, 2, TX_NO_TIME_SLICE, TX_AUTO_START);
}