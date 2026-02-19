#ifndef FMP_CORE_HPP
#define FMP_CORE_HPP

#include <a113/ucp/core.hpp>

#include <esp_log.h>

namespace fmp {

struct env_snapshot_t {
    uint16_t   soil_moisture;
    float      temperature_1;
    float      temperature_2;
    float      pressure;
    float      humidity;
};

};

#endif