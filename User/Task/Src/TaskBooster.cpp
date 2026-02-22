#include "main.h"
#include "tx_api.h"

extern TX_THREAD ControlThread;
extern uint8_t ControlThreadStack[2048];

#define TX_NAME(s) const_cast<CHAR*>(s)
extern "C" void TaskBooster(void)
{

}