#include "pyro_core_def.h"
#include "pyro_core_config.h"
#include "FreeRTOS.h"
#include "task.h"

//声明 C 链接函数（给 FreeRTOS 任务用）
extern "C"
{
extern void pyro_init_thread(void *argument);
}

//任务规划器
extern "C" void start_mission_planer_task(void const *argument)
{
    xTaskCreate(
        pyro_init_thread,
        "pyro_init_thread",
        512,
        nullptr,
        configMAX_PRIORITIES - 1,
        nullptr
    );
    vTaskDelete(nullptr);
}
