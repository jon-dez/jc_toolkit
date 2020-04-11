#pragma once
#include <string>
#include "jctool_types.h"

using namespace System;

template <typename T> T CLAMP(const T& value, const T& low, const T& high)
{
    return value < low ? low : (value > high ? high : value);
}

s16 uint16_to_int16(u16 a);
void decode_stick_params(u16 *decoded_stick_params, u8 *encoded_stick_params);
void encode_stick_params(u8 *encoded_stick_params, u16 *decoded_stick_params);

std::string get_sn(u32 offset, const u16 read_len);
int get_spi_data(u32 offset, const u16 read_len, u8 *test_buf);
int write_spi_data(u32 offset, const u16 write_len, u8* test_buf);
int get_device_info(u8* test_buf);
int get_battery(u8* test_buf);
int get_temperature(u8* test_buf);
int dump_spi(const char *dev_name);
int send_rumble();
int play_tune(int tune_no);
int play_hd_rumble_file(int file_type, u16 sample_rate, int samples, int loop_start, int loop_end, int loop_wait, int loop_times);
int send_custom_command(u8* arg);
int device_connection();
int set_led_busy();
int button_test();
int ir_sensor(ir_image_config &ir_cfg);
int ir_sensor_config_live(ir_image_config &ir_cfg);
int nfc_tag_info();
int silence_input_report();

extern int  handle_ok;
extern bool enable_button_test;
extern bool enable_IRVideoPhoto;
extern bool enable_IRAutoExposure;
extern bool enable_NFCScanning;
extern bool cancel_spi_dump;
extern bool check_connection_ok;

extern u8 timming_byte;
extern u8 ir_max_frag_no;

namespace CppWinFormJoy {
    class images
    {
        //For annoying designer..
        //Todo.
    };
}