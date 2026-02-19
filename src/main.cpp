#include "core.hpp"
#include "env_handler.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

fmp::Env_Handler G_env_handler{ {
    .soil_moisture_rounds = 32
} };

extern "C" void app_main( void ) {
    vTaskDelay( pdMS_TO_TICKS( 3000 ) );

    G_env_handler.init();

    for(;;) {
        fmp::env_snapshot_t ss = {};
        G_env_handler.make_snapshot( &ss );

        printf( "%.2f[*C], %.2f[*C], %.2f[Pa], %.2f[%%]\n", 
            ss.temperature_1, ss.temperature_2, ss.pressure, ss.humidity
        );

        vTaskDelay( pdMS_TO_TICKS( 1000 ) );
    }
}