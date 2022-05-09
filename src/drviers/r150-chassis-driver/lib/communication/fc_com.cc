#include "fc_com.h"

#include <memory>
#include <iostream>
#include <map>
#include <fstream>
#include "fc_uart_queue.h"

#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <queue>

using namespace xag_chassis;
using namespace xag_chassis::communication;
using namespace xag_chassis::common;

std::queue<MessageWrapper> message_received_queue;

API_Config_t FCComComponent::fc_com_config_ = {0};

// 用于superx_ii_api使用，封装性变差
// 用不具名命名空间将全局变量局部化
namespace
{
    FCUartQueue fc_uart_queue_;
} // namespace

void api_port_send(int32_t len, uint8_t *data)
{
    if (data == NULL)
    {
        return;
    }
    if (superx_fd_uart == -1)
    {
        return;
    }
    // // LOG_F(INFO, "api_port_send len is: %d", len);
    // 将要发送给飞控的数据写入队列
    fc_uart_queue_.Enqueue(data, len);
}

void api_on_data_returned(int32_t offset, int32_t len, uint8_t *data)
{
    //
}

void api_on_modulelist_received(uint8_t *data)
{
    //
}

void api_on_message_received(API_Message_t message)
{

    MessageWrapper _message_wrapper(message);
    message_received_queue.push(message);
}

FCComComponent::FCComComponent(uint32_t rate)
{
    loop_duration_ = 1000 / rate; // ms/rate
    lls_offset_flag_ = false;
    lls_offset_ = Eigen::Vector3i::Zero();

    odom_orientation_ = Eigen::Vector3d::Zero();
    odom_position_ = Eigen::Vector3d::Zero();
}

FCComComponent::~FCComComponent()
{
    cmdCloseUart();
}

bool FCComComponent::initialize()
{
    if (cmdTaskInit(config::XAG_RTK_UART_PATH) < 0)
    {
        // LOG_F(WARNING, "FCComComponent: cmdTaskInit failed!");
        return false;
    }
    // LOG_F(INFO, "FCComComponent: cmdTaskInit succeeded!");
    loop_timer_ = 0;

    return true;
}

int FCComComponent::cmdTaskInit(const std::string &uart_path)
{
    if (superx_fd_uart != 0)
        return -1;

    superx_fd_uart = OpenUartCom(uart_path.c_str());

    if (superx_fd_uart == -1)
    {
        cmdCloseUart();
        return -1;
    }

    if (ConfigUartCom(superx_fd_uart, 115200, 8, 1, 'n') < 0)
    {
        cmdCloseUart();
        return -1;
    }

    // uuid_t _module_uuid;
    // uuid_generate(_module_uuid);
    // 设置模块版本号
    {
        std::lock_guard<std::mutex> _lock(module_config_mutex_);
        module_config_.software_version = xag_chassis::config::XAG_RTK_SOFTWARE_VERSION;
        module_config_.hardware_version = xag_chassis::config::XAG_RTK_HARDWARE_VERSION;
    }

    FlushComCacheBuf(superx_fd_uart);
    // 用uuid来做全球唯一标识
    // memcpy((uint8_t*)fc_com_config_.ModuleID, (uint8_t*)_module_uuid + 4, 12);
    fc_com_config_.ModuleID[0] = 0x3B3C3569;
    fc_com_config_.ModuleID[1] = 0x4E24A469;
    fc_com_config_.ModuleID[2] = 0x5f2466D4;
    fc_com_config_.ModuleType = MODULE_TYPE_DEBUGGER;

    fc_com_config_.ModuleTimeout = 1000;
    fc_com_config_.DataSize = 0;
    fc_com_config_.bautrate = xag_chassis::config::XAG_BAUTRATE_OPTION; //5-230400,6-460800,7-576000,8-1000000,9-1152000
    fccom_bautrate = xag_chassis::config::XAG_BAUTRATE_TABLE[xag_chassis::config::XAG_BAUTRATE_OPTION];
    api_init(&fc_com_config_);

    // LOG_F(INFO, "uart init success! configure baudrate uart fd = %d,current baudrate=%d", superx_fd_uart, GetPortSpeed(superx_fd_uart));
    // module_auto_send_data(MODULE_TYPE_OA, 0, 20, 1, 4); // Cycle,ms
    return 0;
}

void FCComComponent::cmdCloseUart(void)
{
    if (superx_fd_uart > 0)
    {
        close_uart(superx_fd_uart);
        superx_fd_uart = 0;
    }
}

bool FCComComponent::process()
{
    static uint32_t _loop_count = 0;
    loop_timer_ += loop_duration_;

    cmdHandleTask(loop_timer_);

    // std::cout <<  (int)fc_com_config_.Status << std::endl;

    if (_loop_count % 10 == 0)
    {
        static bool _once_online;
        if (_once_online && !fcComOnline())
        {
            printf("configure baudrate uart fd = %d,current baudrate=%d\n", superx_fd_uart, GetPortSpeed(superx_fd_uart));
            api_on_configurate_baudrate(superx_fd_uart, 115200);
            // 暂时容忍你
            ::usleep(10000);
        }
        // 通知dls，已经注册上
        if (!_once_online && fcComOnline())
        {
        }
        _once_online = fcComOnline();
        api_loop(loop_timer_);
    }
    ++_loop_count;
}

void FCComComponent::cmdHandleTask(uint32_t _timer)
{
    int data_len;
    if (superx_fd_uart <= 0)
    {
        // LOG_F(WARNING, "superx_fd_uart <= 0 couldn't communicate with fc!");
        return;
    }

    // 没数据的时候会容易越界？
    data_len = UartRead(superx_fd_uart, serial_buffer_, sizeof(serial_buffer_), 1);
    if (data_len > 0)
    {
        api_port_received(data_len, serial_buffer_);
    }
    // 将其他模块发送过来的message发送出去
    // {
    //     std::lock_guard<std::mutex> _lock(send_message_mutex_);
    //     while (!send_message_vec_.empty())
    //     {
    //         // // LOG_F(INFO, "Pop from send message vec!");
    //         auto _message = send_message_vec_.back();
    //         if(_message->message_.Data != nullptr)
    //         {
    //             api_send_message(&(_message->message_));
    //         }
    //         send_message_vec_.pop_back();
    //     }
    // }

    if (!fc_uart_queue_.Empty())
    {
        uint8_t write_uart_buffer[xag_chassis::config::UNIT_WRITE_UART_SIZE];
        int _data_len = fc_uart_queue_.Dequeue(write_uart_buffer, xag_chassis::config::UNIT_WRITE_UART_SIZE);
        if (_data_len > 0)
        {
            UartWrite(superx_fd_uart, write_uart_buffer, _data_len, 1);
        }
    }
}

void FCComComponent::sendCmd(const uint8_t _cmd, const API_ModuleType_t _to, const char _message)
{
    MessageWrapper _message_wrapper;
    _message_wrapper.messageForSend(_cmd, MODULE_TYPE_FC, _message);
    if (fcComOnline())
    {
        api_send_message(&(_message_wrapper.message_));
    }
}

void FCComComponent::sendControlCmd(const uint8_t _cmd, const API_ModuleType_t _to, const uint8_t _subcmd, const R150_Control_Cmd_t _message)
{
    //std::cout << "R150_Control_Cmd_t.speed: " << _message.speed << std::endl;
    //std::cout << "R150_Control_Cmd_t.yawRate: " << _message.yawRate << std::endl;

    MessageWrapper _message_wrapper;
    _message_wrapper.sdkMessageForSend(_cmd, _subcmd, MODULE_TYPE_FC, _message);
    /*for (int i = 0; i < 15; i++)
        printf("%x ", *(_message_wrapper.message_.Data + i));
    printf("\n");*/
    if (fcComOnline())
    {
        api_send_message(&(_message_wrapper.message_));
    }
}

void FCComComponent::sendCmdForStruct(const uint8_t _cmd, const API_ModuleType_t _to, const Fd_Request_Multiple_t _message)
{
    MessageWrapper _message_wrapper;
    _message_wrapper.messageForSend(_cmd, MODULE_TYPE_FC, _message);
    // for (int i = 0; i < 8; i++)
    //     printf("%x ", *(_message_wrapper.message_.Data + i));
    // printf("\n");
    if (fcComOnline())
    {
        api_send_message(&(_message_wrapper.message_));
    }
}

void FCComComponent::sendCmdNoMsg(const uint8_t _cmd, const API_ModuleType_t _to)
{
    MessageWrapper _message_wrapper;
    _message_wrapper.noMessageForSend(_cmd, MODULE_TYPE_FC, 0x00);
    if (fcComOnline())
    {
        api_send_message(&(_message_wrapper.message_));
    }
}

void FCComComponent::getOdomFromDataWarehouse()
{
    if (!message_received_queue.empty())
    {
        MessageWrapper _msg;
        _msg = message_received_queue.front();

        uint8_t _cmd;
        memcpy((void *)&_cmd, _msg.message_.Data, 1);

        if (_cmd == 0x42)
        {
            memcpy((void *)&system_time_, _msg.message_.Data + 2 + 8, 4);
            int16_t _roll_angle; //单位为0.1度
            memcpy((void *)&_roll_angle, _msg.message_.Data + 2 + 8 + 4, 2);
            int16_t _pitch_angle; //单位为0.1度
            memcpy((void *)&_pitch_angle, _msg.message_.Data + 2 + 8 + 6, 2);
            int16_t _yaw_angle; //单位为0.1度
            memcpy((void *)&_yaw_angle, _msg.message_.Data + 2 + 8 + 8, 2);
            // 待完善
            // int32_t _odom_x;//单位为mm
            // memcpy((void *)&_odom_x,_msg.message_.Data+2+8+10,4);
            // int32_t _odom_y;//单位为mm
            // memcpy((void *)&_odom_y,_msg.message_.Data+2+8+14,4);

            odom_orientation_[0] = double(_roll_angle) / 573;
            odom_orientation_[1] = double(_pitch_angle) / 573;
            odom_orientation_[2] = double(_yaw_angle) / 573;
            // 待完善
            // odom_position_[0] = double(_odom_x)/1000;
            // odom_position_[1] = double(_odom_y)/1000;
            // odom_position_[2] = 0.0;

            //std::cout << "_yaw_angle: "<<_yaw_angle << std::endl;
        }
        message_received_queue.pop();
    }
}
