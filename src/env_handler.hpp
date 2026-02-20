#ifndef FMP_ENV_HANDLER_HPP
#define FMP_ENV_HANDLER_HPP

#include "core.hpp"
#include "bridge.hpp"

#include <driver/i2c_master.h>

#include <a113/ucp/espressif32/IO_i2c.hpp>
#include <a113/ucp/sensor_drivers/AHT21.hpp>
#include <a113/ucp/sensor_drivers/BMP280.hpp>

#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>

namespace fmp {

class Env_Handler {
public:
    struct config_t {
        uint8_t       soil_moisture_rounds   = 0;
        uint32_t      main_task_stack        = 4096;    
        UBaseType_t   main_task_priority     = configMAX_PRIORITIES - 1;
        uint32_t      measure_period_ms      = 1000;
    };

public:
    Env_Handler( const config_t& config_  ) : _config{ config_ } {}

    ~Env_Handler( void ) {
        if( _h_adc1 ) adc_oneshot_del_unit( _h_adc1 ); 
        _h_adc1 = NULL;

        if( _h_adc1_2_cal ) adc_cali_delete_scheme_curve_fitting( _h_adc1_2_cal );
        _h_adc1_2_cal = NULL;
    }

_A113_PROTECTED:
    inline static const adc_oneshot_unit_init_cfg_t _ADC_UNIT_1_CONFIG = {
        .unit_id  = ADC_UNIT_1,
        .clk_src  = ADC_DIGI_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };

    inline static const adc_oneshot_chan_cfg_t _ADC_SOIL_MOISTURE_CHANNEL_CONFIG = {
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12
    };

    inline static const adc_cali_curve_fitting_config_t _ADC_SOIL_MOISTURE_CAL_CONFIG = {
        .unit_id  = ADC_UNIT_1,
        .chan     = ADC_CHANNEL_2,
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12
    };

    inline static const i2c_master_bus_config_t _I2C_BUS_0_CONFIG = {
        .i2c_port          = I2C_NUM_0,
        .sda_io_num        = GPIO_NUM_8,
        .scl_io_num        = GPIO_NUM_9,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .trans_queue_depth = 0x0, // Unused.
        .flags             = {
            .enable_internal_pullup = false,
            .allow_pd               = false
        }
    };

    inline static const i2c_device_config_t _I2C_DEV_AHT21_CONFIG = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = a113::sdrv::AHT21::I2C_ADDRESS,
        .scl_speed_hz    = 100000,
    };

    inline static const i2c_device_config_t _I2C_DEV_BMP280_CONFIG = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = a113::sdrv::BMP280::I2C_ADDRESS_SDO_VCC,
        .scl_speed_hz    = 100000,
    };

public:
    a113::status_t init( void ) {
    /* Soil moisture sensor. */
        adc_oneshot_new_unit( &_ADC_UNIT_1_CONFIG, &_h_adc1 );
        adc_oneshot_config_channel( _h_adc1, ADC_CHANNEL_2, &_ADC_SOIL_MOISTURE_CHANNEL_CONFIG );
        adc_cali_create_scheme_curve_fitting( &_ADC_SOIL_MOISTURE_CAL_CONFIG, &_h_adc1_2_cal );

    /* BMP280. */
        i2c_new_master_bus( &_I2C_BUS_0_CONFIG, &_h_i2c_bus );
        _i2c_aht21.bind( _h_i2c_bus, _I2C_DEV_AHT21_CONFIG, 50 );
        _i2c_bmp280.bind( _h_i2c_bus, _I2C_DEV_BMP280_CONFIG, 50 );

        _aht21.bind_i2c( &_i2c_aht21 );
        _aht21.calib();

        _bmp280.bind_i2c( &_i2c_bmp280 );
        _bmp280.load_calibs();

    /* Read task. */
        xTaskCreate(
            &Env_Handler::_main, "fmp::Env_Handler::_main", 
            _config.main_task_stack, ( void* )this, _config.main_task_priority, 
            &_h_main 
        );

        return A113_OK;
    }

protected:
    config_t                    _config         = {};

    TaskHandle_t                _h_main         = NULL;

    adc_oneshot_unit_handle_t   _h_adc1         = NULL;
    adc_cali_handle_t           _h_adc1_2_cal   = NULL;

    i2c_master_bus_handle_t     _h_i2c_bus      = NULL;
    a113::esp32::io::I2C_m2s    _i2c_aht21      = {};
    a113::esp32::io::I2C_m2s    _i2c_bmp280     = {};
    a113::sdrv::AHT21           _aht21          = {};
    a113::sdrv::BMP280          _bmp280         = {};

protected:
    static void _main( void* arg_ ) { 
        auto* self = ( Env_Handler* )arg_;    
    for(;;) {
        vTaskDelay( pdMS_TO_TICKS( self->_config.measure_period_ms ) );

        env_snapshot_t ss;
        self->_soil_moisture_read( &ss.soil_moisture );
        self->_temp_hum_read( &ss.temperature_1, &ss.humidity );
        self->_temp_press_read( &ss.temperature_2, &ss.pressure );

        Bridge.commit_env_ss( ss );
    } }

protected:
    a113::status_t _soil_moisture_read( uint16_t* sm_ ) {
        uint32_t acc = 0;
        for( uint8_t s = 1; s <= _config.soil_moisture_rounds; ++s ) {
            int raw; adc_oneshot_read( _h_adc1, ADC_CHANNEL_2, &raw );
            acc += raw; 
        }
        acc /= _config.soil_moisture_rounds;

        int voltage = 0x0;
        adc_cali_raw_to_voltage( _h_adc1_2_cal, acc, &voltage );
        *sm_ = ( uint16_t )voltage;

        return A113_OK;
    }

    a113::status_t _temp_hum_read( float* temp_, float* hum_ ) {
        using namespace a113::sdrv;

        A113_ASSERT_STATUS_OR_RET( _aht21.one_shot() );
        vTaskDelay( pdMS_TO_TICKS( 60 ) );
        return _aht21.load_data( temp_, hum_ );
    }

    a113::status_t _temp_press_read( float* temp_, float* press_ ) {
        using namespace a113::sdrv;

        A113_ASSERT_STATUS_OR_RET(
            _bmp280.store_ctrl_meas( BMP280::CtrlMeas_TemperatureSampling_1x | BMP280::CtrlMeas_PressureSampling_1x | BMP280::CtrlMeas_Power_OneShot )
        );
        vTaskDelay( pdMS_TO_TICKS( 10 ) );
        _bmp280.load_data( temp_, press_ );
        _bmp280.store_ctrl_meas( BMP280::CtrlMeas_Power_Low );
        return A113_OK;
    }

};

};

#endif