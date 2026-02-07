#include "core.hpp"
#include "env_handler.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

fmp::Env_Handler G_env_handler{ {
    .soil_moisture_rounds = 32
} };

extern "C" void app_main( void ) {
    G_env_handler.init();

    for(;;) {
        fmp::env_snapshot_t ss = {};
        G_env_handler.make_snapshot( &ss );

        printf( "%u.%03u | %d.%02u degrees\n", ss.soil_moisture/1000, ss.soil_moisture%1000, ss.temperature/100, ss.temperature%100 );

        vTaskDelay( pdMS_TO_TICKS( 1000 ) );
    }
}