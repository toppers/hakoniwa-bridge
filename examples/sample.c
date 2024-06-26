#include "hako_asset.h"
#include <stdio.h>
#include <stdlib.h>
#include "std_msgs/pdu_ctype_Bool.h"
#include "geometry_msgs/pdu_ctype_Twist.h"
#include "pdu_primitive_ctypes.h"

#ifdef _WIN32
static inline void usleep(long microseconds) {
    Sleep(microseconds / 1000);
}
#else
#include <unistd.h>
#endif

#define PDU_POS_CHANNEL_ID          0
#define PDU_BAGGAGE_CHANNEL_ID      1
#define PDU_BUMPER_CHANNEL_ID       2

static int my_on_initialize(hako_asset_context_t* context)
{
    printf("INFO: my_on_initialize enter\n");
    printf("INFO: sleep 1sec\n");
    usleep(1000*1000);
    printf("INFO: my_on_initialize exit\n");
    return 0;
}
static int my_on_reset(hako_asset_context_t* context)
{
    printf("INFO: my_on_reset enter\n");
    printf("INFO: sleep 1sec\n");
    usleep(1000*1000);
    printf("INFO: my_on_reset exit\n");
    return 0;
}
static int my_on_simulation_step(hako_asset_context_t* context)
{
    char pos_buffer[HAKO_PDU_FIXED_DATA_SIZE(sizeof(Hako_Twist))];
    char bumper_buffer[HAKO_PDU_FIXED_DATA_SIZE(sizeof(Hako_Bool))];
    char baggage_buffer[HAKO_PDU_FIXED_DATA_SIZE(sizeof(Hako_Bool))];
    static int send_value = 0;
    Hako_Twist *pos;
    Hako_Bool bumper;
    Hako_Bool baggage;

    printf("INFO: on_simulation_step enter: %llu\n", hako_asset_simulation_time());
    bumper.data = send_value;
    if (hako_pdu_put_fixed_data(bumper_buffer, (const char*)&bumper, sizeof(bumper), sizeof(bumper_buffer)) != 0) {
        printf("ERROR: hako_pdu_put_fixed_data error\n");
    }
    int ret = hako_asset_pdu_write("RobotAvator", PDU_BUMPER_CHANNEL_ID, (const char*)(&bumper_buffer), sizeof(bumper_buffer));
    if (ret != 0) {
        printf("ERROR: hako_asset_pdu_write erro: %d\n", ret);
    }
    baggage.data = !send_value;
    if (hako_pdu_put_fixed_data(baggage_buffer, (const char*)&baggage, sizeof(baggage), sizeof(baggage_buffer)) != 0) {
        printf("ERROR: hako_pdu_put_fixed_data error\n");
    }
    ret = hako_asset_pdu_write("RobotAvator", PDU_BAGGAGE_CHANNEL_ID, (const char*)(&baggage_buffer), sizeof(baggage_buffer));
    if (ret != 0) {
        printf("ERROR: hako_asset_pdu_write erro: %d\n", ret);
    }
    send_value = !send_value;

    ret = hako_asset_pdu_read("RobotAvator", PDU_POS_CHANNEL_ID, (char*)(&pos_buffer), sizeof(pos_buffer));
    if (ret != 0) {
        printf("ERROR: hako_asset_pdu_read erro: %d\n", ret);
    }
    pos = (Hako_Twist*)hako_get_base_ptr_pdu(pos_buffer);
    if (pos != NULL) {
        printf("%llu: pos data(%f, %f, %f)\n", hako_asset_simulation_time(), pos->linear.x, pos->linear.y, pos->linear.z);
    }

    usleep(1000*1000);
    printf("INFO: on_simulation_step exit\n");
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
    if (argc != 4) {
        printf("Usage: %s <asset_name> <config_path> <delta_time_msec>\n", argv[0]);
        return 1;
    }
    const char* asset_name = argv[1];
    const char* config_path = argv[2];
    hako_time_t delta_time_usec = atoi(argv[3]) * 1000;

    int ret = hako_asset_register(asset_name, config_path, &my_callback, delta_time_usec, HAKO_ASSET_MODEL_PLANT);
    if (ret != 0) {
        printf("ERORR: hako_asset_register() returns %d.", ret);
        return 1;
    }
    ret = hako_asset_start();
    printf("INFO: hako_asset_start() returns %d\n", ret);

    return 0;
}
