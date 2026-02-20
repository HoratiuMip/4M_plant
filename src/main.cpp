#include "core.hpp"
#include "env_handler.hpp"
#include "fx_handler.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

fmp::Env_Handler G_env_handler{ {
    .soil_moisture_rounds = 32,
    .main_task_stack      = 4096,
    .main_task_priority   = fmp::Priority_3,
    .measure_period_ms    = 3000
} };

fmp::FX_Handler G_fx_handler{ {
    .Q_moisture_indicator_led = GPIO_NUM_7
}, {
    .moisture_indicator_blink_period_ms = 8000,
    .main_task_stack                    = 4096,
    .main_task_priority                 = fmp::Priority_2
} };

extern "C" void app_main( void ) {
    vTaskDelay( pdMS_TO_TICKS( 3000 ) );
    G_env_handler.init();
    G_fx_handler.init();
}