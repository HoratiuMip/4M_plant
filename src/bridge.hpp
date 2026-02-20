#ifndef FMP_BRIDGE_HPP
#define FMP_BRIDGE_HPP

#include "core.hpp"

namespace fmp {

class _Bridge {
public:
    _Bridge( void ) {
        
    }

protected:
    env_snapshot_t   _latest_env_ss   = {};
    portMUX_TYPE     _env_ss_splck    = portMUX_INITIALIZER_UNLOCKED;

public:
    void commit_env_ss( const env_snapshot_t& ss_ ) {
        taskENTER_CRITICAL( _env_ss_splck );
        _latest_env_ss = ss_;
        taskEXIT_CRITICAL( _env_ss_splck );
    }

    env_snapshot_t get_env_ss( void ) {
        taskENTER_CRITICAL( _env_ss_splck );
        auto ss = _latest_env_ss;
        taskEXIT_CRITICAL( _env_ss_splck );
        return ss;
    }

}; inline _Bridge Bridge;

};

#endif