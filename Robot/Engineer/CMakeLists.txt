
# =========================================================================
# Engineer 工程车底盘代码库
# =========================================================================
#set(BOARD "ARM_BOARD")
set(BOARD "CHASSIS_BOARD")
add_library(Engineer OBJECT
)

target_compile_definitions(Engineer PRIVATE
        BOARD=${BOARD}
)

target_compile_options(Engineer PRIVATE
    -mcpu=cortex-m7
    -mthumb
    -mfpu=fpv5-d16
    -mfloat-abi=hard
)
set(BASE_SOURCES
)

set(ARM_EXTRA_SOURCES
)
set(CHASSIS_EXTRA_SOURCES
#==================== 任务模块 =====================
pyro_init_thread.cpp
pyro_mission_planner.cpp
#==================== 底盘模块 =====================
Chassis/pyro_engineer_chassis.cpp
Chassis/pyro_chassis_leg.cpp
Chassis/pyro_engineer_chassis_mecanum.cpp
Chassis/pyro_engineer_chassis_mecanum_power.cpp
Chassis/pyro_engineer_chassis_mecanum_supercap.cpp
Chassis/pyro_engineer_chassis_track.cpp
#==================== 状态机模块 =====================
Chassis/fsm/pyro_engineer_active_state.cpp
Chassis/fsm/pyro_engineer_passive_state.cpp
Chassis/fsm/leg/pyro_engineer_leg_disabled_state.cpp
Chassis/fsm/leg/pyro_engineer_leg_uncalibrated_state.cpp
Chassis/fsm/leg/pyro_engineer_leg_vmc_state.cpp
Chassis/fsm/leg/pyro_engineer_leg_pid_gravity_state.cpp
Chassis/fsm/leg/pyro_engineer_leg_pid_home_state.cpp
#==================== 应用模块 =====================
Application/Chassis_board/pyro_chassis_app.cpp
#==================== 调试模块 =====================
Application/Chassis_board/pyro_chassis_debug.cpp
#==================== 通信模块 =====================
Communication/Chassis_board/pyro_chassis_remote.cpp
)
if (BOARD STREQUAL "ARM_BOARD")
    target_sources(Engineer PRIVATE ${BASE_SOURCES} ${ARM_EXTRA_SOURCES}
    )
    target_compile_definitions(PYRo PUBLIC
            DR16_UART=PYRO_UART5
    )
    target_compile_definitions(Engineer PRIVATE
            DR16_UART=PYRO_UART5
            SELF_CTRL_UART=PYRO_UART10
            DEBUG_UART=PYRO_UART1
            INTERBOARD_UART=PYRO_UART7
    )
elseif (BOARD STREQUAL "CHASSIS_BOARD")
    target_sources(Engineer PRIVATE ${BASE_SOURCES} ${CHASSIS_EXTRA_SOURCES})
    # SUPERCAP_UART 同时作用于 PYRo 库内的驱动实现和 Engineer 侧调用。
    target_compile_definitions(PYRo PUBLIC
    DR16_UART=PYRO_UART5
    SUPERCAP_UART=PYRO_UART7
    )
    target_compile_definitions(Engineer PRIVATE
    DR16_UART=PYRO_UART5
    SUPERCAP_UART=PYRO_UART7
    #SUPER_CAP_UART=PYRO_UART1
    DEBUG_UART=PYRO_UART1
    #这两个UART灵活变通一下
    #REFEREE_UART=PYRO_UART7
    #INTERBOARD_UART=PYRO_UART10
    )
else()
        message(FATAL_ERROR "Unknown BOARD type: ${BOARD}. Please set BOARD to either ARM_BOARD or CHASSIS_BOARD.")
endif()

# ===================== 头文件包含路径 =====================
target_include_directories(Engineer
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}                        # Engineer 根目录
        ${CMAKE_CURRENT_SOURCE_DIR}/Chassis                 # 底盘模块头文件
        ${CMAKE_CURRENT_SOURCE_DIR}/Application/Chassis_board  # 应用装配
        ${CMAKE_CURRENT_SOURCE_DIR}/Communication/Chassis_board  # 遥控器通信

)

# ===================== 链接依赖库 =====================
target_link_libraries(Engineer
        PUBLIC
        stm32cubemx
        dsppp_lib
        PYRo
)
