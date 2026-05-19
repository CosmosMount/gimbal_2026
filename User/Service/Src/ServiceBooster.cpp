//
// Created by cosmosmount on 2025/9/8.
//

#include "main.h"
#include "tx_api.h"
#include "led.hpp"

extern TX_THREAD RemoterThread;
extern TX_SEMAPHORE RemoterGot;
extern TX_SEMAPHORE RemoterGotVT;
extern uint8_t RemoterThreadStack[1024];
extern void RemoterThreadFun(ULONG initial_input);

extern TX_THREAD IMUThread;
extern TX_SEMAPHORE IMUThreadSem;
extern uint8_t IMUThreadStack[4096];
extern void IMUThreadFun(ULONG initial_input);

extern TX_THREAD IMUTempThread;
extern uint8_t IMUTempThreadStack[1024];
extern void IMUTempThreadFun(ULONG initial_input);

extern TX_THREAD AliveThread;
extern uint8_t AliveThreadStack[512];
extern void AliveThreadFun(ULONG thread_input);
extern TX_SEMAPHORE CANErrorSem;

/*EKF pool*/
TX_BYTE_POOL KFPool;
UCHAR KF_PoolBuf[4096] = {0};

/*OneMessage pool*/
TX_BYTE_POOL MsgPool;
UCHAR Msg_PoolBuf[4096] = {0};

#define TX_NAME(s) const_cast<CHAR*>(s)

extern "C" void ServiceBooster()
{
    /*Math pool in ccram*/
    tx_byte_pool_create(
            &KFPool,
            (CHAR *) "KF_Pool",
            KF_PoolBuf,
            sizeof(KF_PoolBuf));

    tx_byte_pool_create(
            &MsgPool,
            (CHAR *) "Msg_Pool",
            Msg_PoolBuf,
            sizeof(Msg_PoolBuf));

    tx_thread_create(&AliveThread, TX_NAME("AliveThread"),
        AliveThreadFun, 0x1234, AliveThreadStack, sizeof(AliveThreadStack),
        10, 10, TX_NO_TIME_SLICE, TX_AUTO_START);

    tx_thread_create(&RemoterThread, TX_NAME("RemoterThread"),
        RemoterThreadFun, 0x1234, RemoterThreadStack, sizeof(RemoterThreadStack),
        2, 2, TX_NO_TIME_SLICE, TX_AUTO_START);

    tx_semaphore_create(&RemoterGot, TX_NAME("RemoterGot"), 0);
    tx_semaphore_create(&RemoterGotVT, TX_NAME("RemoterGotVT"), 0);
    
    tx_thread_create(&IMUThread, TX_NAME("IMUThread"),
        IMUThreadFun, 0x1234, IMUThreadStack, sizeof(IMUThreadStack),
        3, 3, TX_NO_TIME_SLICE, TX_AUTO_START);

    tx_thread_create(&IMUTempThread, TX_NAME("IMUTempThread"),
        IMUTempThreadFun, 0x1234, IMUTempThreadStack, sizeof(IMUTempThreadStack),
        4, 4, TX_NO_TIME_SLICE, TX_AUTO_START);

    tx_semaphore_create(&IMUThreadSem, TX_NAME("IMUThreadSem"), 0);
    tx_semaphore_create(&CANErrorSem, TX_NAME("CANErrSem"), 0);
}