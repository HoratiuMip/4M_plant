#ifndef FMP_CORE_HPP
#define FMP_CORE_HPP

#include <a113/ucp/core.hpp>

#include <freertos/FreeRTOS.h>
#include <esp_log.h>

namespace fmp {

enum Priority_ : UBaseType_t {
    Priority_1 = 0x1,
    Priority_2 = 0x2,
    Priority_3 = 0x3
};

struct env_snapshot_t {
    uint16_t   soil_moisture;
    float      temperature_1;
    float      temperature_2;
    float      pressure;
    float      humidity;
};

};

#endif