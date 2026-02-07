#ifndef FMP_ENV_HANDLER_HPP
#define FMP_ENV_HANDLER_HPP

#include "core.hpp"

#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>

#include <driver/i2c_master.h>

#include <freertos/FreeRTOS.h>
#include <freertos/ringbuf.h>

namespace fmp {

class Env_Handler {
public:
    struct config_t {
        uint8_t   soil_moisture_rounds   = 0;
    };

public:
    Env_Handler( const config_t& config_  ) : _config{ config_ } {}

    ~Env_Handler( void ) {
        if( _h_adc1 ) adc_oneshot_del_unit( _h_adc1 ); 
        _h_adc1 = NULL;

        if( _h_adc1_2_cal ) adc_cali_delete_scheme_curve_fitting( _h_adc1_2_cal );
        _h_adc1_2_cal = NULL;
        
        if( _ring ) vRingbufferDelete( _ring );
        _ring = NULL;
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
        .sda_io_num        = GPIO_NUM_6,
        .scl_io_num        = GPIO_NUM_7,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .trans_queue_depth = 0x0, // Unused.
        .flags             = {
            .enable_internal_pullup = false,
            .allow_pd               = false
        }
    };

    inline static const i2c_device_config_t _I2C_DEV_BMP280_CONFIG = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = 0x76,
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
        i2c_master_bus_add_device( _h_i2c_bus, &_I2C_DEV_BMP280_CONFIG, &_h_i2c_bmp280 );

        this->_load_bmp280_calibs();

    /* Ring. */
        _ring = xRingbufferCreate( 64, RINGBUF_TYPE_NOSPLIT );

        return A113_OK;
    }

_A113_PROTECTED:
    config_t                        _config             = {};

    adc_oneshot_unit_handle_t       _h_adc1             = NULL;
    adc_cali_handle_t               _h_adc1_2_cal       = NULL;

    i2c_master_bus_handle_t         _h_i2c_bus          = NULL;
    i2c_master_dev_handle_t         _h_i2c_bmp280       = NULL;
    struct _bmp280_calibs_t {
        uint16_t   dig_T1   = 0x0;
        int16_t    dig_T2   = 0x0;
        int16_t    dig_T3   = 0x0;
        uint16_t   dig_P1   = 0x0;
        int16_t    dig_P2   = 0x0;
        int16_t    dig_P3   = 0x0;
        int16_t    dig_P4   = 0x0;
        int16_t    dig_P5   = 0x0;
        int16_t    dig_P6   = 0x0;
        int16_t    dig_P7   = 0x0;
        int16_t    dig_P8   = 0x0;
        int16_t    dig_P9   = 0x0;
    }                                _bmp280_calibs      = {};
    int32_t                          _bmp280_fine_temp   = 0x0;

    RingbufHandle_t                  _ring               = NULL;

_A113_PROTECTED:
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

_A113_PROTECTED:
    a113::status_t _load_bmp280_calibs( void ) {
        static const int BMP280_TIMEOUT_MS = 30;

        uint8_t buffer[ 24 ];

        i2c_master_transmit_receive( _h_i2c_bmp280, (uint8_t[]){ 0x88 }, 1, buffer, sizeof( buffer ), BMP280_TIMEOUT_MS );
    #define _DML_MAKE_WORD( offset ) ( ( buffer[ offset+1 ] << 8 ) | buffer[ offset ] )
        _bmp280_calibs.dig_T1 = _DML_MAKE_WORD(0);
        _bmp280_calibs.dig_T2 = _DML_MAKE_WORD(2);
        _bmp280_calibs.dig_T3 = _DML_MAKE_WORD(4);
        _bmp280_calibs.dig_P1 = _DML_MAKE_WORD(6);
        _bmp280_calibs.dig_P2 = _DML_MAKE_WORD(8);
        _bmp280_calibs.dig_P3 = _DML_MAKE_WORD(10);
        _bmp280_calibs.dig_P4 = _DML_MAKE_WORD(12);
        _bmp280_calibs.dig_P5 = _DML_MAKE_WORD(14);
        _bmp280_calibs.dig_P6 = _DML_MAKE_WORD(16);
        _bmp280_calibs.dig_P7 = _DML_MAKE_WORD(18);
        _bmp280_calibs.dig_P8 = _DML_MAKE_WORD(20);
        _bmp280_calibs.dig_P9 = _DML_MAKE_WORD(22);
    #undef _DML_MAKE_WORD

        return A113_OK;
    }

    a113::status_t _temp_press_read( int16_t* temp_, uint32_t* press_ ) {
        static const int BMP280_TIMEOUT_MS   = 20;
        static const int BMP280_INIT_HOLD_MS = 20;

        i2c_master_transmit( _h_i2c_bmp280, (uint8_t[]){ 0xF4, 0b00100111 }, 2, BMP280_TIMEOUT_MS );
        i2c_master_transmit( _h_i2c_bmp280, (uint8_t[]){ 0xF5, 0b00000000 }, 2, BMP280_TIMEOUT_MS );
        vTaskDelay( BMP280_INIT_HOLD_MS );
        
        uint8_t buffer[ 6 ];

        i2c_master_transmit_receive( _h_i2c_bmp280, (uint8_t[]){ 0xF7 }, 1, buffer, sizeof( buffer ), BMP280_TIMEOUT_MS );
  
        int32_t raw_temp  = ( int32_t )( ( buffer[ 3 ] << 12 ) | ( buffer[ 4 ] << 4 ) | ( buffer[ 5 ] >> 4 ) );
        int64_t raw_press = ( int32_t )( ( buffer[ 0 ] << 12 ) | ( buffer[ 1 ] << 4 ) | ( buffer[ 2 ] >> 4 ) );

    {
        int32_t var1, var2;

        var1 = ((((raw_temp>>3) - ((int32_t)_bmp280_calibs.dig_T1<<1))) * ((int32_t)_bmp280_calibs.dig_T2)) >> 11;
        var2 = (((((raw_temp>>4) - ((int32_t)_bmp280_calibs.dig_T1)) * ((raw_temp>>4) - ((int32_t)_bmp280_calibs.dig_T1))) >> 12) * ((int32_t)_bmp280_calibs.dig_T3)) >> 14;

        _bmp280_fine_temp = var1 + var2;
        raw_temp          = (_bmp280_fine_temp * 5 + 128) >> 8;

        *temp_ = (int16_t)raw_temp;
    }
    {
        int64_t var1, var2;

        var1 = ((int64_t)_bmp280_fine_temp) - 128000;
        var2 = var1 * var1 * (int64_t)_bmp280_calibs.dig_P6;
        var2 = var2 + ((var1*(int64_t)_bmp280_calibs.dig_P5)<<17);
        var2 = var2 + (((int64_t)_bmp280_calibs.dig_P4)<<35);
        var1 = ((var1 * var1 * (int64_t)_bmp280_calibs.dig_P3)>>8) + ((var1 * (int64_t)_bmp280_calibs.dig_P2)<<12);
        var1 = (((((int64_t)1)<<47)+var1))*((int64_t)_bmp280_calibs.dig_P1)>>33;

        if(0 != var1) {
            raw_press = 1048576-raw_press;
            raw_press = (((raw_press<<31)-var2)*3125)/var1;
            var1      = (((int64_t)_bmp280_calibs.dig_P9) * (raw_press>>13) * (raw_press>>13)) >> 25;
            var2      = (((int64_t)_bmp280_calibs.dig_P8) * raw_press) >> 19;
            raw_press = ((raw_press + var1 + var2) >> 8) + (((int64_t)_bmp280_calibs.dig_P7)<<4);
            
            *press_ = (uint32_t)raw_press;
        } 
    }
        i2c_master_transmit( _h_i2c_bmp280, (uint8_t[]){ 0xF4, 0b00000000 }, 2, BMP280_TIMEOUT_MS );
        return A113_OK;
    }

public:
    a113::status_t make_snapshot( env_snapshot_t* ss_ ) {
        this->_soil_moisture_read( &ss_->soil_moisture );
        this->_temp_press_read( &ss_->temperature, &ss_->pressure );

        return A113_OK;
    }

};

};

#endif