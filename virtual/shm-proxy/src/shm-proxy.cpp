#include "hako_asset.h"
#include "hako_conductor.h"
#include <stdio.h>
#include <stdlib.h>
#ifdef _WIN32
static inline void usleep(long microseconds) {
    Sleep(microseconds / 1000);
}
#else
#include <unistd.h>
#endif

static int my_on_initialize(hako_asset_context_t* context)
{
    //TODO
    return 0;
}
static int my_on_reset(hako_asset_context_t* context)
{
    //TODO
    return 0;
}
static int my_on_simulation_step(hako_asset_context_t* context)
{
    //TODO
    return 0;
}


static hako_asset_callbacks_t my_callback = {
    .on_initialize = my_on_initialize,
    .on_manual_timing_control = NULL,
    .on_simulation_step = my_on_simulation_step,
    .on_reset = my_on_reset
};
int main(int argc, const char* argv[])
{
    if ((argc != 4) && (argc != 5)) {
        printf("Usage: %s <asset_name> <config_path> <delta_time_msec> [master]\n", argv[0]);
        return 1;
    }
    const char* asset_name = argv[1];
    const char* config_path = argv[2];
    bool enable_master = false;
    hako_time_t delta_time_usec = atoi(argv[3]) * 1000;
    if (argc == 5)
    {
        enable_master = true;
    }

    if (enable_master) {
        hako_conductor_start(delta_time_usec, delta_time_usec);
    }
    int ret = hako_asset_register(asset_name, config_path, &my_callback, delta_time_usec, HAKO_ASSET_MODEL_CONTROLLER);
    if (ret != 0) {
        printf("ERORR: hako_asset_register() returns %d.", ret);
        return 1;
    }
    ret = hako_asset_start();

    if (enable_master) {
        hako_conductor_stop();
    }
    return 0;
}
