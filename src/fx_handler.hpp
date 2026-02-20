#ifndef FMP_FX_HANDLER_HPP
#define FMP_FX_HANDLER_HPP

#include "core.hpp"
#include "config.hpp"
#include "bridge.hpp"

#include <driver/gpio.h>

namespace fmp {

class FX_Handler {
public:
    struct pin_map_t {
        gpio_num_t   Q_moisture_indicator_led   = GPIO_NUM_NC;
    };
    struct config_t {
        uint32_t      moisture_indicator_blink_period_ms   = 8000;
        uint32_t      main_task_stack                      = 4096;
        UBaseType_t   main_task_priority                   = configMAX_PRIORITIES - 1;
    };

public:
    FX_Handler( const pin_map_t& pin_map_, const config_t& config_ ) : _pin_map{ pin_map_ }, _config{ config_ } {}

public:
    a113::status_t init( void ) {
        gpio_config_t indicator_cfg = {
            .pin_bit_mask = 0x1ULL << _pin_map.Q_moisture_indicator_led,
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE
        };
        gpio_config( &indicator_cfg );

        xTaskCreate(
            &FX_Handler::_main, "fmp::FX_Handler::_main",
            _config.main_task_stack, ( void* )this, _config.main_task_priority,
            &_h_main
        );

        return A113_OK; 
    }

protected:
    pin_map_t      _pin_map   = {};
    config_t       _config    = {};

    TaskHandle_t   _h_main    = NULL;

protected:
    static void _main( void* arg_ ) {
        auto* self = ( FX_Handler* )arg_;
    for(;;) {
        vTaskDelay( pdMS_TO_TICKS( self->_config.moisture_indicator_blink_period_ms ) );

        auto ss      = Bridge.get_env_ss();
        auto percent = self->_wet_percent( ss.soil_moisture );

        for( int b = 1; b <= percent / 20 + 1; ++b ) {
            gpio_set_level( self->_pin_map.Q_moisture_indicator_led, HIGH );
            vTaskDelay( pdMS_TO_TICKS( 100 ) );
            gpio_set_level( self->_pin_map.Q_moisture_indicator_led, LOW );
            vTaskDelay( pdMS_TO_TICKS( 100 ) );
        }
    } }

protected:
    int _wet_percent( uint16_t volt_ ) {
        if( volt_ < MOISTURE_VOLTAGE_WET ) return 100;
        if( volt_ > MOISTURE_VOLTAGE_DRY ) return 0;

        return 100 - (volt_ - MOISTURE_VOLTAGE_WET) * 100 / (MOISTURE_VOLTAGE_DRY - MOISTURE_VOLTAGE_WET);
    }
};

};

#endif