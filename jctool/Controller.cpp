/*

MIT License

Copyright (c) 2020 Jonathan Mendez

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/
#include "Controller.hpp"

#include "jctool.h"
#include "jctool_helpers.hpp"
#include "ir_sensor.h"
#include "ImageLoad/ImageLoad.hpp"

#ifdef __linux__
#include <cstring> // memset linux
#include <unistd.h>
#endif

#include "TP/TP.hpp"

Controller::IRSensor::IRSensor(){
    this->capture_status.message_stream << "IR Message:" << std::endl;
    memset(&this->config, 0, sizeof(this->config));
    ir_image_config& ir_config = this->config;
    ir_config.ir_leds_intensity = ~ir_config.ir_leds_intensity; // Max intensity.
    ir_config.ir_ex_light_filter = ir_ex_light_fliter_0;
    ir_config.ir_denoise = ir_denoise_Enable;
    ir_config.ir_digital_gain = 1;

    ir_image_config_Sets::exposure(ir_config.ir_exposure, 300);
    ir_image_config_Sets::denoise_edge_smooth(ir_config.ir_denoise, 35);
    ir_image_config_Sets::denoise_color_intrpl(ir_config.ir_denoise, 68);
}

/**
 * Updates the battery information with the value read from the controller.
 */
void Controller::updateBatteryData(){
    unsigned char battery_data[3];
    memset(battery_data, 0, sizeof(battery_data));
    get_battery(this->hid_handle, this->timming_byte, battery_data);

    this->battery = parseBatteryData(battery_data);
}

void Controller::updateTemperatureData(){
    unsigned char temperature_data[2];
    memset(temperature_data, 0, sizeof(temperature_data));
    get_temperature(this->hid_handle, this->timming_byte, temperature_data);
    
    this->temperature = parseTemperatureData(temperature_data);
}

void Controller::connection(){
    int handle_ok = 0;
    if((handle_ok = device_connection(this->hid_handle)) == 0) {// TODO: , this->hid_serial_number)) == 0)
#ifdef __linux__
        memcpy(this->device_info, "NONE", 5);
#else
        memcpy_s(this->device_info, 10, "NONE", 5);
#endif
        this->controller_type = Controller::Type::None;
        this->hid_handle = nullptr;
        return;
    }
    else {
        get_device_info(this->hid_handle, this->timming_byte, reinterpret_cast<unsigned char*>(this->device_info));
        this->updateBatteryData();
        this->updateTemperatureData();

        this->serial_number = get_sn(this->hid_handle, this->timming_byte);

        // Get the controller colors
        this->saved_colors = this->preview_colors = get_spi_colors(this->hid_handle, this->timming_byte);
    }

    // Set the controller type based on the return value.
    switch (handle_ok)
    {
        case 1:
            this->controller_type = Controller::Type::JoyConLeft;
            break;
        case 2:
            this->controller_type = Controller::Type::JoyConRight;
            break;
        case 3:
            this->controller_type = Controller::Type::ProCon;
            break;
        default:
            this->controller_type = Controller::Type::Undefined;
            break;
    }
}

void Controller::IRSensor::capture(controller_hid_handle_t host_controller, u8& timming_byte){
    if(host_controller == nullptr)
        return; // There is no controller
    
    this->capture_in_progress = true;
    // Set the max frag number.
    switch(this->config.ir_res_reg) {
        case IR_320x240:
            this->ir_max_frag_no = 0xff;
            break;
        case IR_160x120:
            this->ir_max_frag_no = 0x3f;
            break;
        case IR_80x60:
            this->ir_max_frag_no = 0x0f;
            break;
        case IR_40x30:
            this->ir_max_frag_no = 0x03;
            break;
    }
    
    IRCaptureCTX capture_context{
            host_controller,
            timming_byte,
            this->config,
            this->ir_max_frag_no,
            this->capture_mode,
            this->capture_status
    };
    int res = ir_sensor(capture_context,
        [this](const u8* raw_capture, size_t size_raw_capture){
            auto raw_capture_copy = std::shared_ptr<u8>(new u8[size_raw_capture], [](u8* d){ delete[] d;});
            memcpy(raw_capture_copy.get(), raw_capture, size_raw_capture);
            GPUTexture::SideLoader::add_job(
                [this, raw_capture_copy](){
                    auto& resolution = std::get<2>(ir_resolutions[this->res_idx_selected]);
                    ImageResourceData ird;
                    ird.width = resolution.width;
                    ird.height = resolution.height;
                    ird.num_channels = 3;
                    ird.bytes = new u8[ird.width*ird.height*ird.num_channels];

                    // Colorize the raw capture.
                    colorizefrom8BitsPP(
                        raw_capture_copy.get(),
                        ird.bytes,
                        ird.width,
                        ird.height,
                        ird.num_channels,
                        this->colorize_with,
                        ColorOrder::InRGBOrder
                    );

                    std::lock_guard<std::mutex> lock(this->vstream_frame_dat.texture_mutex);
                    this->vstream_frame_dat.updated = true;
                    // 1. Render to the texture.
                    GPUTexture::openGLUpload(this->vstream_frame_dat.textures[this->vstream_frame_dat.idx_render], ird.width, ird.height, ird.num_channels, ird.bytes);
                    // 2. The texture was rendered so put it in the swap,
                    // so that the new texture could be swapped out for the old texture.
                    std::swap(this->vstream_frame_dat.idx_swap, this->vstream_frame_dat.idx_render);
                }
            );
        }
    );
    this->capture_in_progress = false;
    if(res > 0)
        this->capture_status.message_stream << ir_sensorErrorToString(res);
}

uintptr_t Controller::IRSensor::getCaptureTexID() {
    auto& frame_dat = this->vstream_frame_dat;
    std::lock_guard<std::mutex> lock(frame_dat.texture_mutex);
    if(frame_dat.updated){
        // 3. Get the new texture from the swap, and replace the swap with the old texture.
        std::swap(frame_dat.idx_swap, frame_dat.idx_display);
        frame_dat.updated = false;
        GPUTexture::SideLoader::add_job([this](){
            // 4. Free the old texture from the swap.
            GPUTexture::openGLFree(this->vstream_frame_dat.textures[this->vstream_frame_dat.idx_swap]);
        });
    }
    return frame_dat.textures[frame_dat.idx_display];
}

void Controller::rumble(RumbleData& rumble_data){
    this->rumble_active = true;
    int res = play_hd_rumble_file(this->hid_handle, this->timming_byte, rumble_data);
    this->rumble_active = false;
}
