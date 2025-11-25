#include "app_main.h"
#include "app/chassis/chassis.hpp"
#include "app/robotcmd/robotcmd.hpp"
#include "bsp/system/mutex.hpp"
#include "cmsis_os.h"

#include "bsp/log/log.hpp"
#include "bsp/system/time.hpp"
#include "bsp/uart/stm32_uart.hpp"

#include "gimbal/gimbal.hpp"
#include "modules/imu/ins_task.h"
#include "modules/motor/motor.hpp"
#include "modules/motor/DJIMotor/DJIMotor.hpp"
#include "modules/motor/DMMotor/DMMotor.hpp"
#include <memory>
#include <string>

// 全局变量：电机实例（GM6020/J4310/J8009）
std::shared_ptr<DJIMotor> gm6020 = nullptr;
std::shared_ptr<DMMotor> j4310 = nullptr;
std::shared_ptr<DMMotor> j8009 = nullptr;
std::shared_ptr<float> gimbal_yaw_motor_angle_ptr = nullptr;
std::shared_ptr<RobotCMD> robotcmd = nullptr;
std::shared_ptr<Chassis> chassis = nullptr;
std::shared_ptr<Gimbal> gimbal = nullptr;
attitude_t* imu_data = nullptr;

// 新增：串口句柄和接收缓冲区
UARTHandle_t usart1 = nullptr;
std::string serial_buf;

// 原有任务声明
void RobotCMDTask(void* arg __attribute__((unused)));
void ChassisTask(void* arg __attribute__((unused)));
void INSTask(void* arg __attribute__((unused)));
void GimbalTask(void* arg __attribute__((unused)));

// 新增：串口指令解析任务和回调函数
void SerialParseTask(void* arg __attribute__((unused)));
void uart1_callback(uint8_t* data, uint32_t len);  // 修改回调函数参数

void app_main()
{
    taskENTER_CRITICAL();
    RTTLog::init();
    STM32TimeDWT::DWT_Init(168);
    LOGINFO("Robot", "System Init");
    STM32CAN_Init();
    StartDaemonTask();
    StartMotorControlTask();

    // 初始化串口（用于接收指令，例如USART1）
    extern UART_HandleTypeDef huart1;
    usart1 = STM32UART_Init(&huart1, uart1_callback);  // 注册接收回调

    // 初始化电机实例
    extern CAN_HandleTypeDef hcan1;

    // 1. GM6020电机（位置控制模式，带软件限位）
    gm6020 = std::make_shared<DJIMotor>(
        &hcan1,
        0x201,
        MotorType::GM6020,
        MotorPIDSetting{
            .outer_loop = CloseloopType::ANGLE_LOOP,
            // 修正枚举位运算，添加显式转换
            .close_loop = static_cast<CloseloopType>(
                static_cast<uint8_t>(CloseloopType::ANGLE_LOOP) | 
                static_cast<uint8_t>(CloseloopType::SPEED_LOOP) | 
                static_cast<uint8_t>(CloseloopType::CURRENT_LOOP)
            ),
            .reverse = false,
            .external_angle_feedback = FeedbackType::INTERNAL,
            .external_speed_feedback = FeedbackType::INTERNAL
        },
        MotorPID{
            .pid_angle_ = PIDController(PIDConfig{8.0f, 0.1f, 0.2f, 360.0f, 5.0f}),
            .pid_speed_ = PIDController(PIDConfig{4.5f, 0.0f, 0.1f, 1000.0f, 10.0f}),
            .pid_current_ = PIDController(PIDConfig{0.5f, 0.0f, 0.0f, 20000.0f, 500.0f})
        }
    );
    gm6020->setPositionParams(-180.0f, 180.0f);
    gm6020->enable();

    // 2. J4310电机（使用现有可用的MotorType，如M2006）
    j4310 = std::make_shared<DMMotor>(
        &hcan1,
        0x202,
        0x000,
        MotorType::M2006,  // 修正为现有类型
        MotorPIDSetting{
            .outer_loop = CloseloopType::ANGLE_LOOP,
            .close_loop = CloseloopType::ANGLE_AND_SPEED_LOOP,
            .reverse = false,
            .external_angle_feedback = FeedbackType::INTERNAL,
            .external_speed_feedback = FeedbackType::INTERNAL
        },
        MotorPID{
            .pid_angle_ = PIDController(PIDConfig{5.0f, 0.0f, 0.1f, 360.0f, 3.0f}),
            .pid_speed_ = PIDController(PIDConfig{3.0f, 0.0f, 0.05f, 800.0f, 8.0f})
        }
    );
    j4310->setPositionParams(-90.0f, 90.0f);
    j4310->enable();

    // 3. J8009电机（使用现有可用的MotorType，如M3508）
    j8009 = std::make_shared<DMMotor>(
        &hcan1,
        0x203,
        0x000,
        MotorType::M3508,  // 修正为现有类型
        MotorPIDSetting{
            .outer_loop = CloseloopType::ANGLE_LOOP,
            .close_loop = CloseloopType::ANGLE_AND_SPEED_LOOP,
            .reverse = false,
            .external_angle_feedback = FeedbackType::INTERNAL,
            .external_speed_feedback = FeedbackType::INTERNAL
        },
        MotorPID{
            .pid_angle_ = PIDController(PIDConfig{6.0f, 0.0f, 0.1f, 360.0f, 4.0f}),
            .pid_speed_ = PIDController(PIDConfig{3.5f, 0.0f, 0.05f, 900.0f, 9.0f})
        }
    );
    j8009->setPositionParams(0.0f, 360.0f);
    j8009->enable();

    // 原有模块初始化
    gimbal_yaw_motor_angle_ptr = std::make_shared<float>(0.0f);
    chassis = std::make_shared<Chassis>();
    imu_data = INS_Init();
    robotcmd = std::make_shared<RobotCMD>(imu_data, gimbal_yaw_motor_angle_ptr);
    gimbal = std::make_shared<Gimbal>(imu_data, gimbal_yaw_motor_angle_ptr);

    // 创建原有任务
    xTaskCreate(RobotCMDTask, "RobotCMDTask", 1024, nullptr, osPriorityNormal, nullptr);
    xTaskCreate(ChassisTask, "ChassisTask", 1024, nullptr, osPriorityNormal, nullptr);
    xTaskCreate(INSTask, "INSTask", 1024, nullptr, osPriorityNormal, nullptr);
    xTaskCreate(GimbalTask, "GimbalTask", 1024, nullptr, osPriorityNormal, nullptr);

    // 新增：创建串口指令解析任务
    xTaskCreate(SerialParseTask, "SerialParseTask", 1024, nullptr, osPriorityNormal, nullptr);

    taskEXIT_CRITICAL();
    LOGINFO("Robot", "Robot Init Completed");
}

// 修正：串口接收回调函数参数
void uart1_callback(uint8_t* data, uint32_t len)
{
    if (data != nullptr && len > 0)
    {
        for (uint32_t i = 0; i < len; i++)
        {
            if (data[i] == '\n')  // 指令以换行符结束
            {
                // 不处理，等待解析任务处理
            }
            else
            {
                serial_buf += data[i];  // 累加接收的字符
            }
        }
    }
}

// 新增：串口指令解析任务
void SerialParseTask(void* arg __attribute__((unused)))
{
    while (true)
    {
        if (!serial_buf.empty())
        {
            std::string cmd = serial_buf;
            serial_buf.clear();

            // 解析指令格式："电机类型:目标角度"
            size_t colon_pos = cmd.find(':');
            if (colon_pos == std::string::npos)
            {
                LOGERROR("Serial", "Invalid command format: " + cmd);
                continue;
            }

            std::string motor_type = cmd.substr(0, colon_pos);
            std::string angle_str = cmd.substr(colon_pos + 1);
            float target_angle = std::stof(angle_str);

            // 根据电机类型设置目标角度
            if (motor_type == "GM6020" && gm6020)
            {
                gm6020->setRef(target_angle);  // 使用setRef而非setTargetAngle
                LOGINFO("Serial", "GM6020 set to " + std::to_string(target_angle) + "°");
            }
            else if (motor_type == "J4310" && j4310)
            {
                j4310->setRef(target_angle);
                LOGINFO("Serial", "J4310 set to " + std::to_string(target_angle) + "°");
            }
            else if (motor_type == "J8009" && j8009)
            {
                j8009->setRef(target_angle);
                LOGINFO("Serial", "J8009 set to " + std::to_string(target_angle) + "°");
            }
            else
            {
                LOGERROR("Serial", "Unknown motor type: " + motor_type);
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// 原有任务实现（保持不变）
void RobotCMDTask(void* arg __attribute__((unused)))
{
    while (true)
    {
        robotcmd->RobotCMDTask();
        vTaskDelay(2 / portTICK_PERIOD_MS);
    }
}

void ChassisTask(void* arg __attribute__((unused)))
{
    while (true)
    {
        chassis->ChassisTask();
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}

void INSTask(void* arg __attribute__((unused)))
{
    while (true)
    {
        INS_Task();
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void GimbalTask(void* arg __attribute__((unused)))
{
    while (true)
    {
        gimbal->GimbalTask();
        vTaskDelay(2 / portTICK_PERIOD_MS);
    }
}