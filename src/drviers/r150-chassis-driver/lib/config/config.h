#pragma once

#include <string>
#include <limits>
#include "communication/superx_ii_api/superx_ii_api.h"

namespace xag_chassis
{
namespace config
{

const std::string XAG_RTK_UART_PATH = "/dev/ttyUSB0";

const uint32_t XAG_RTK_SOFTWARE_VERSION = VERSION(0,0,0);
const uint32_t XAG_RTK_HARDWARE_VERSION = VERSION(0,0,0);

const uint8_t  XAG_BAUTRATE_OPTION   = 0;

const uint32_t XAG_BAUTRATE_TABLE[10] = {115200, 0, 0, 57600, 0, 256000, 460800, 576000, 1000000, 1152000};

// 单次从uart中读出的最大字节数
const size_t UNIT_READ_UART_SIZE  = 1000;
// 单次写入uart的最大字节数
const size_t UNIT_WRITE_UART_SIZE = 1000;

}
}
