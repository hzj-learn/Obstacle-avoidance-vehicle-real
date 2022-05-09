/*
 * @Description: 
 * @Version: 0.0
 * @Autor: LQF
 * @Date: 2020-09-15 11:51:43
 * @LastEditors: LQF
 * @LastEditTime: 2020-09-23 09:41:37
 */
#pragma once

#include <mutex>
#include <vector>
#include <iostream>

#include "superx_ii_api/superx_ii_api.h"
#include "communication/fc_uart_queue.h"
#include "config/config.h"
#include "message_wrapper/message_wrapper.h"
#include "common/convert/convert.h"

#define PRINT_RECIEVE_MSG true

namespace xag_chassis
{
    namespace communication
    {

        struct ModuleConfig_t
        {
            ModuleConfig_t() {}
            uint32_t software_version;
            uint32_t hardware_version;
        };

        struct Fd_Request_Multiple_t
        {
            uint8_t subcmd; // 子命令0x02
            uint8_t number; // 结构体数量
            uint16_t reserved1;
            uint16_t id; // 命令识别号
            uint16_t reserved2;
            uint16_t offset[20]; // 偏移地址，支持20个
            uint8_t length[20];  // 数据长度，支持20个
            uint8_t type[20];    // 结构体编号，支持20个
        };

        struct R150_Control_Cmd_t
        {
            uint8_t controlFlag; // 0:切出控制权（需遥控器先解除权限），超1秒自动切出
            uint8_t Rev1[3];
            int32_t speed;   // 1mm/s
            int32_t yawRate; // 0.1度
        };

        class FCComComponent
        {
        public:
            FCComComponent(uint32_t rate);
            ~FCComComponent();
            bool initialize();
            // 主流程
            bool process();

            // 初始化与飞控的通信
            int cmdTaskInit(const std::string &uart_path);
            // 处理与飞控的通信
            void cmdHandleTask(uint32_t _timer);
            void cmdCloseUart(void);

            void sendCmd(const uint8_t cmd, const API_ModuleType_t _to, const char _message);
            void sendControlCmd(const uint8_t _cmd, const API_ModuleType_t _to, const uint8_t _sub_cmd, const R150_Control_Cmd_t _message);
            void sendCmdNoMsg(const uint8_t _cmd, const API_ModuleType_t _to);
            void sendCmdForStruct(const uint8_t _cmd, const API_ModuleType_t _to, const Fd_Request_Multiple_t _message);
            void getOdomFromDataWarehouse();

            uint32_t sysTime() { return system_time_; }
            Eigen::Vector3d odomOrientation() { return odom_orientation_; }
            Eigen::Vector3d odomPosition() { return odom_position_; }

            static bool fcComOnline(void)
            {
                switch (fc_com_config_.Status)
                {
                case MODULE_STATUS_OFFLINE:
                {
                    return false;
                    break;
                }
                case MODULE_STATUS_ONLINE:
                {
                    return true;
                    break;
                }
                }
            }

        private:
            // 用于接受fc数据的buffer
            uint8_t serial_buffer_[xag_chassis::config::UNIT_READ_UART_SIZE];
            static API_Config_t fc_com_config_;

            uint32_t loop_timer_;
            double loop_duration_;

            std::mutex send_message_mutex_;
            std::vector<std::shared_ptr<MessageWrapper>> send_message_vec_;

            std::mutex module_config_mutex_;
            ModuleConfig_t module_config_;

            bool lls_offset_flag_;
            Eigen::Vector3i lls_offset_;

            uint32_t system_time_;

            Eigen::Vector3d odom_orientation_;
            Eigen::Vector3d odom_position_;
        };

    } // namespace communication
} // namespace xag_chassis
