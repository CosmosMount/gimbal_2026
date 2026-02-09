//
// Created by cosmosmount on 2025/9/8.
//

#include "main.h"
#include "tx_api.h"
#include "led.hpp"

TX_THREAD my_thread1;
uint8_t my_thread_stack1[512];
TX_SEMAPHORE my_semaphore1;

TX_THREAD my_thread2;
uint8_t my_thread_stack2[512];

extern TX_THREAD RemoterThread;
extern TX_SEMAPHORE RemoterGot;
extern uint8_t RemoterThreadStack[1024];
extern void RemoterThreadFun(ULONG initial_input);

extern TX_THREAD IMUThread;
extern TX_SEMAPHORE IMUThreadSem;
extern uint8_t IMUThreadStack[4096];
extern void IMUThreadFun(ULONG initial_input);

extern TX_THREAD IMUTempThread;
extern uint8_t IMUTempThreadStack[1024];
extern void IMUTempThreadFun(ULONG initial_input);

extern TX_THREAD RefereeThread;
extern uint8_t RefereeThreadStack[2048];
extern void RefereeThreadFun(ULONG initial_input);

/*EKF pool*/
TX_BYTE_POOL KFPool;
UCHAR KF_PoolBuf[4096] = {0};

/*OneMessage pool*/
TX_BYTE_POOL MsgPool;
UCHAR Msg_PoolBuf[4096] = {0};

[[noreturn]] void my_thread_entry(ULONG thread_input)
{
    LED_ALL_ON();
    /* Enter into a forever loop. */
    while(1)
    {

        /* Increment thread counter. */
        tx_semaphore_put(&my_semaphore1);
        /* Sleep for 1 tick. */
        tx_thread_sleep(500);
    }
}

[[noreturn]] void my_thread_entry2(ULONG thread_input)
{
    /* Enter into a forever loop. */
    while(1)
    {
        /* Increment thread counter. */
        if (tx_semaphore_get(&my_semaphore1, TX_WAIT_FOREVER) == TX_SUCCESS)
        {
            LED_blink();
            /* Sleep for 1 tick. */
        }
    }
}

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

    tx_semaphore_create(&my_semaphore1, TX_NAME("my_semaphore1"), 0);

    /* Create my_thread! */
    tx_thread_create(&my_thread1, TX_NAME("my_thread1"),
        my_thread_entry, 0x1234, my_thread_stack1, sizeof(my_thread_stack1),
        10, 10, TX_NO_TIME_SLICE, TX_AUTO_START);

    tx_thread_create(&my_thread2, TX_NAME("my_thread2"),
        my_thread_entry2, 0x1234, my_thread_stack2, sizeof(my_thread_stack2),
        10, 10, TX_NO_TIME_SLICE, TX_AUTO_START);

    tx_thread_create(&RemoterThread, TX_NAME("RemoterThread"),
        RemoterThreadFun, 0x1234, RemoterThreadStack, sizeof(RemoterThreadStack),
        2, 2, TX_NO_TIME_SLICE, TX_AUTO_START);

    tx_semaphore_create(&RemoterGot, TX_NAME("RemoterGot"), 0);

    tx_thread_create(&IMUThread, TX_NAME("IMUThread"),
        IMUThreadFun, 0x1234, IMUThreadStack, sizeof(IMUThreadStack),
        3, 3, TX_NO_TIME_SLICE, TX_AUTO_START);

    tx_semaphore_create(&IMUThreadSem, TX_NAME("IMUThreadSem"), 0);

    tx_thread_create(&IMUTempThread, TX_NAME("IMUTempThread"),
        IMUTempThreadFun, 0x1234, IMUTempThreadStack, sizeof(IMUTempThreadStack),
        4, 4, TX_NO_TIME_SLICE, TX_AUTO_START);

    tx_thread_create(&RefereeThread, TX_NAME("RefereeThread"),
        RefereeThreadFun, 0x1234, RefereeThreadStack, sizeof(RefereeThreadStack),
        8, 8, TX_NO_TIME_SLICE, TX_AUTO_START);
}