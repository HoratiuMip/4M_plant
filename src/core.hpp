#ifndef FMP_CORE_HPP
#define FMP_CORE_HPP

#include <a113/ucp/core.hpp>

namespace fmp {

struct env_snapshot_t {
    uint16_t   soil_moisture;
    int16_t    temperature;
    uint32_t   pressure;
};

};

#endif