#include "pyro_bsp_uart.h"
#include "pyro_bsp_can.h"
#include "pyro_dwt_drv.h"
#include "pyro_dr16_rc_drv.h"
#include "FreeRTOS.h"
#include "task.h"
#include "pyro_referee.h"

namespace pyro{

/*==================全局驱动指针========================*/
// CAN 驱动指针
can_drv_t *can1_drv = nullptr;
can_drv_t *can2_drv = nullptr;
can_drv_t *can3_drv = nullptr;

extern "C" void chassis_app_init(void);
extern "C" void chassis_remote_init(void);

extern "C"{
void pyro_init_thread(void *argument){
    (void)argument;

    // DWT 定时器初始化（480MHz 主频）
    dwt_drv_t::init(480);

    /*      UART 初始化（DR16 专用 UART5 在下方单独配置）      */
    bsp_uart::get_uart1().enable_rx_dma();
    bsp_uart::get_uart7().enable_rx_dma();
    bsp_uart::get_uart10().enable_rx_dma();

    /*       CAN 初始化            */
    // CAN1
    can1_drv = &bsp_can::get_can1();
    can1_drv->init();
    can1_drv->start();

    // CAN2
    can2_drv = &bsp_can::get_can2();
    can2_drv->init();
    can2_drv->start();

    // CAN3：4 个 M3508 轮电机
    can3_drv = &bsp_can::get_can3();
    can3_drv->init();
    can3_drv->start();

#ifdef REFEREE_UART
    REFEREE_UART.reset(115200, UART_WORDLENGTH_8B, UART_STOPBITS_1,
                       UART_PARITY_NONE);
    REFEREE_UART.enable_rx_dma();
    referee_drv_t::get_instance()->init();
#endif

    /*      DR16 遥控器（UART5，100000 波特率、9 位、2 停止位、偶校验）      */
#ifdef DR16_UART
    dr16_drv_t::instance().start();
    dr16_drv_t::instance().enable();
    DR16_UART.reset(100000, UART_WORDLENGTH_9B, UART_STOPBITS_2,
                    UART_PARITY_EVEN);
    DR16_UART.enable_rx_dma();
#endif

    /*      底盘模块装配（电机/PID 依赖、模块启动）      */
    chassis_app_init();

    /*      遥控器通信任务（DR16 读遥控并下发命令）      */
    chassis_remote_init();

    vTaskDelete(nullptr);
}
}
}
