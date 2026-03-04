#include "DMMotor.hpp"
#include "led.hpp"
#include "stm32h7xx_hal_def.h"
#include "tx_api.h"
#include "DJIMotorHandler.hpp"

TX_THREAD AliveThread;
uint8_t AliveThreadStack[512] = {0};

/* Semaphores for Alivecheck */
extern TX_SEMAPHORE IMUThreadSem;
extern TX_SEMAPHORE CANErrorSem;

[[noreturn]] void AliveThreadFun(ULONG thread_input)
{
    UNUSED(thread_input);
    LED_ALL_ON();

    for(;;)
    {
        bool imu_alive = tx_semaphore_get(&IMUThreadSem, TX_NO_WAIT) == TX_SUCCESS;
        bool can_error = tx_semaphore_get(&CANErrorSem, TX_NO_WAIT) == TX_SUCCESS;
        if (can_error)
        {
            LED_blink(LED_COLOR::LED_RED);
        }
        else if (!imu_alive) 
        {
            LED_blink(LED_COLOR::LED_WHITE);        
        }
        else
        {
            if (DJIMotorHandler::Instance()->AllMotorAlive())
            {
                LED_blink(LED_COLOR::LED_GREEN);
            }
            else
            {
                LED_blink(LED_COLOR::LED_BLUE);
            }
        }
        tx_thread_sleep(2);
    }
}