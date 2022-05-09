#include "communication/message_wrapper/message_wrapper.h"
#include <iostream>

using namespace xag_chassis;
using namespace xag_chassis::communication;

MessageWrapper::MessageWrapper(const API_Message_t& _message)
{
    message_.Data = nullptr;
    modules_list_ = nullptr;
    cmd_ = 0;

    if(_message.Length >= 0)
    {
        message_ = _message;
        uint8_t* __data = new uint8_t[_message.Length];
        memcpy(__data, _message.Data, _message.Length);
        message_.Data = __data;
    }
}

MessageWrapper::MessageWrapper(const uint8_t* _modules_list)
{
    message_.Data = nullptr;
    modules_list_ = nullptr;
    cmd_ = 0;

    if(_modules_list == nullptr)
    {
        return;
    }

    if(_modules_list[0] >= 0)
    {
        uint8_t* __modules_list = new uint8_t[_modules_list[0]*24 + 1];
        memcpy(__modules_list, _modules_list, _modules_list[0]*24 + 1);
        modules_list_ = __modules_list;
    }
}

MessageWrapper::MessageWrapper(const Command _cmd)
{
    message_.Data = nullptr;
    modules_list_ = nullptr;
    cmd_ = 0;
    cmd_ = static_cast<uint32_t>(_cmd);
    // // LOG_F(INFO, "MessageWrapper commond: 0x%x", cmd_);
}

MessageWrapper::MessageWrapper(const MessageWrapper& _wrapper)
{
    message_.Data = nullptr;
    modules_list_ = nullptr;
    cmd_ = 0;

    if(_wrapper.message_.Data != nullptr)
    {
        message_ = _wrapper.message_;
        uint8_t* __data = new uint8_t[_wrapper.message_.Length];
        memcpy(__data, _wrapper.message_.Data, _wrapper.message_.Length);
        message_.Data = __data;
    }
    if(_wrapper.modules_list_ != nullptr)
    {
        uint8_t* __modules_list = new uint8_t[_wrapper.modules_list_[0]*24+1];
        memcpy(__modules_list, _wrapper.modules_list_, _wrapper.modules_list_[0]*24+1);
        modules_list_ = __modules_list;
    }
    else if(_wrapper.cmd_ != 0)
    {
        cmd_ = _wrapper.cmd_;
        // // LOG_F(INFO, "copy MessageWrapper commond: 0x%x", cmd_);
    }
}

void MessageWrapper::requestMessageForSend(const uint8_t cmd, const API_ModuleType_t _to)
{
    if(message_.Data != nullptr)
    {
        delete[] message_.Data;
        message_.Data = nullptr;
    }
    message_.Data = new uint8_t[2];
    message_.Data[0] = cmd;
    message_.Data[1] = (MODULE_TYPE_XLINK << 3) + 1;
    message_.From    = (MODULE_TYPE_XLINK << 3) + 1;
    message_.To      = (_to << 3) + 1;
    message_.Length  = 2;
}

MessageWrapper& MessageWrapper::operator=(const MessageWrapper& _rhs)
{
    if(this == &_rhs)
    {
        return *this;
    }

    // 交换
    MessageWrapper _lhs(_rhs);
    uint8_t* __data = message_.Data;
    uint8_t* __modules_list = modules_list_;
    message_ = _lhs.message_;
    modules_list_ = _lhs.modules_list_;
    cmd_ = _lhs.cmd_;
    _lhs.message_.Data = __data;
    _lhs.modules_list_ = __modules_list;
    // // LOG_F(INFO, "Using the operator= to copy message: %d, CMD is %d", message_.Data, (int)message_.Data[0]);

    return *this;
}

MessageWrapper::~MessageWrapper()
{
    // // LOG_F(INFO, "Release the message!");
    if(message_.Data != nullptr)
    {
        // // LOG_F(INFO, "Realease the message data: %d, CMD is %d", message_.Data, (int)message_.Data[0]);
        delete[] message_.Data;
        message_.Data = nullptr;
    }
    if(modules_list_ != nullptr)
    {
        // // LOG_F(INFO, "Release modules list_!");
        delete[] modules_list_;
        modules_list_ = nullptr;
    }
}

void MessageWrapper::setMessage(const API_Message_t& _message)
{
    // if(message_.Data != nullptr)
    // {
    //     delete[] message_.Data;
    //     message_.Data = nullptr;
    // }
    MessageWrapper _lhs(_message);
    uint8_t* __data = message_.Data;
    message_ = _lhs.message_;
    _lhs.message_.Data = __data;
    // // LOG_F(INFO, "Using set message: %d, CMD is %d", message_.Data, (int)message_.Data[0]);
}


