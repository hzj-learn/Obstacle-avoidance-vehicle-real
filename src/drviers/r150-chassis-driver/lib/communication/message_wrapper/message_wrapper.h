#pragma once

#include <memory>
#include <cstring>
#include "communication/superx_ii_api/superx_ii_api.h"

namespace xag_chassis
{
namespace communication
{

enum class Command
{
    CMD_NOMODULE = 0x00,
    CMD_REGISTER = 0x01,
    CMD_HEARTBEAT = 0x02,
    CMD_GET_MODULE = 0x03,
    CMD_GET_DATA = 0x04,
    CMD_SET_DATA = 0x05,
    CMD_SEND_MSG = 0x06,
    CMD_AUTO_LOAD_DATA = 0x07,
    CMD_UPGRADE = 0xf0,
    CMD_UPGRADE_REQUEST = 0xf1,
    CMD_UPGRADE_PK = 0xf2,
    CMD_UPGRADE_FINISH = 0xf3,

    CMD_FC_ONLINE = 0x8001
};

class MessageWrapper
{
public:
    MessageWrapper() 
    {
        message_.Data = nullptr;
        modules_list_ = nullptr;
        cmd_ = 0;
    }
    MessageWrapper(const API_Message_t& _message);
    MessageWrapper(const MessageWrapper& _wrapper);
    MessageWrapper& operator=(const MessageWrapper& _rhs);
    MessageWrapper(const uint8_t* _modules_list);
    MessageWrapper(const Command _cmd);
    void setMessage(const API_Message_t& _message);

    template <typename MessageT>
    void replyMessage(const MessageWrapper& _wrapper, const MessageT& _message);
    void requestMessageForSend(const uint8_t cmd, const API_ModuleType_t _to);
    template <typename MessageT>
    void messageForSend(uint8_t cmd, const API_ModuleType_t _to, const MessageT& _message);
    template <typename MessageT>
    void noMessageForSend(const uint8_t cmd, const API_ModuleType_t _to, const MessageT& _message);
    template <typename MessageT>
    void sdkMessageForSend(const uint8_t cmd, const uint8_t sub_cmd, const API_ModuleType_t _to, const MessageT& _message);

    virtual ~MessageWrapper();
    API_Message_t message_;
private:
    uint8_t* modules_list_;
    uint32_t cmd_;
    // uint8_t data_[MESSAGE_BUFFER_SIZE];
};

template <typename MessageT>
void MessageWrapper::replyMessage(const MessageWrapper& _wrapper, const MessageT& _message)
{
    if(message_.Data != nullptr)
    {
        delete[] message_.Data;
        message_.Data = nullptr;
    }

    message_.Data = new uint8_t[sizeof(MessageT)+2];

    message_.Data[0] = (uint8_t)_wrapper.message_.Data[0];
    message_.Data[1] = _wrapper.message_.To;
    message_.From    = _wrapper.message_.To;
    message_.version = _wrapper.message_.version;
    message_.To      = _wrapper.message_.From;
    message_.Length  = sizeof(MessageT) + 2;

    memcpy(message_.Data+2, &_message, sizeof(MessageT));
}

template <typename MessageT>
void MessageWrapper::messageForSend(const uint8_t cmd, const API_ModuleType_t _to, const MessageT& _message)
{    
    if(message_.Data != nullptr)
    {
        delete[] message_.Data;
        message_.Data = nullptr;
    }

    message_.Data = new uint8_t[sizeof(MessageT)+3];
    message_.Data[0] = cmd;
    message_.Data[1] = (MODULE_TYPE_DEBUGGER << 3) + 1;
    message_.From    = (MODULE_TYPE_DEBUGGER << 3) + 1;
    message_.To      = (_to << 3) + 1;
    message_.Length  = sizeof(MessageT) + 2;

    memcpy(message_.Data+2, (char*)&_message, sizeof(MessageT));
}

template <typename MessageT>
void MessageWrapper::noMessageForSend(const uint8_t cmd, const API_ModuleType_t _to, const MessageT& _message)
{    
    if(message_.Data != nullptr)
    {
        delete[] message_.Data;
        message_.Data = nullptr;
    }
    message_.Data = new uint8_t[2];
    message_.Data[0] = cmd;
    message_.Data[1] = (MODULE_TYPE_DEBUGGER << 3) + 1;
    message_.From    = (MODULE_TYPE_DEBUGGER << 3) + 1;
    message_.To      = (_to << 3) + 1;
    message_.Length  =  2;

}

template <typename MessageT>
void MessageWrapper::sdkMessageForSend(const uint8_t cmd, const uint8_t sub_cmd, const API_ModuleType_t _to, const MessageT& _message)
{    
    if(message_.Data != nullptr)
    {
        delete[] message_.Data;
        message_.Data = nullptr;
    }
    message_.Data = new uint8_t[sizeof(MessageT)+3];
    message_.Data[0] = cmd;
    message_.Data[1] = (MODULE_TYPE_XLINK << 3) + 1;
    message_.From    = (MODULE_TYPE_XLINK << 3) + 1;
    message_.To      = (_to << 3) + 1;
    message_.Length  = sizeof(MessageT) + 3;

    message_.Data[2] = sub_cmd;
    memcpy(message_.Data+3, (char*)&_message, sizeof(MessageT));
}

}
}
