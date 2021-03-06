#pragma once
#include <memory>
#include <cstdint>
#include <sstream>
#include "hidapi.h"

/**
 * ===============================================================
 * The below code is from the original Joy-Con Toolkit from CTCaer
 * unless mentioned otherwise. There may have been some slight
 * modifications to the original code, but only for convienience.
 * ===============================================================
 */

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
typedef int8_t s8;
typedef int16_t s16;
typedef int32_t s32;
typedef int64_t s64;

#pragma pack(push, 1)

struct brcm_hdr {
    u8 cmd;
    u8 timer;
    u8 rumble_l[4];
    u8 rumble_r[4];
};

struct brcm_cmd_01 {
    u8 subcmd;
    union {
        struct {
            u32 offset;
            u8 size;
        } spi_data;

        struct {
            u8 arg1;
            u8 arg2;
        } subcmd_arg;

        struct {
            u8  mcu_cmd;
            u8  mcu_subcmd;
            u8  mcu_mode;
        } subcmd_21_21;

        struct {
            u8  mcu_cmd;
            u8  mcu_subcmd;
            u8  no_of_reg;
            u16 reg1_addr;
            u8  reg1_val;
            u16 reg2_addr;
            u8  reg2_val;
            u16 reg3_addr;
            u8  reg3_val;
            u16 reg4_addr;
            u8  reg4_val;
            u16 reg5_addr;
            u8  reg5_val;
            u16 reg6_addr;
            u8  reg6_val;
            u16 reg7_addr;
            u8  reg7_val;
            u16 reg8_addr;
            u8  reg8_val;
            u16 reg9_addr;
            u8  reg9_val;
        } subcmd_21_23_04;

        struct {
            u8  mcu_cmd;
            u8  mcu_subcmd;
            u8  mcu_ir_mode;
            u8  no_of_frags;
            u16 mcu_major_v;
            u16 mcu_minor_v;
        } subcmd_21_23_01;
    };
};

struct ir_image_config {
    u8  ir_res_reg;
    u16 ir_exposure;
    u8  ir_leds; // Leds to enable, Strobe/Flashlight modes
    u16 ir_leds_intensity; // MSByte: Leds 1/2, LSB: Leds 3/4
    u8  ir_digital_gain;
    u8  ir_ex_light_filter;
    u32 ir_custom_register; // MSByte: Enable/Disable, Middle Byte: Edge smoothing, LSB: Color interpolation
    u16 ir_buffer_update_time;
    u8  ir_hand_analysis_mode;
    u8  ir_hand_analysis_threshold;
    u32 ir_denoise; // MSByte: Enable/Disable, Middle Byte: Edge smoothing, LSB: Color interpolation
    u8  ir_flip;
};

#pragma pack(pop)

/**
 * ===========================================================================
 * The code below was added for convienience to build off the original Joy-Con
 * Toolkit 
 * ===========================================================================
 */

enum ir_leds_Flags : u8 {
    ir_leds_FlashlightMode = 1,
    ir_leds_WideDisable = 1 << 4,
    ir_leds_NarrowDisable = 1 << 5,
    ir_leds_StrobeFlashMode = 1 << 7
};
enum flip_Flags : u8 {
    flip_dir_0 = 1 << 1
};
enum ir_ex_light_filter_Flags : u8 {
    ir_ex_light_fliter_0 = 0x3
};
enum ir_denoise_Flags : u32 {
    ir_denoise_Enable = 1 << 16
};

struct BatteryData {
    int percent;
    int report;
    float voltage;
};

struct TemperatureData {
    float celsius;
    float fahrenheit;
};


enum VIBType : unsigned char {
    VIBInvalid,
    VIBRaw,
    VIBBinary,
    VIBBinaryLoop,
    VIBBinaryLoopAndWait
};

struct VIBMetadata {
    VIBType vib_file_type;
    u16 sample_rate;
    u32 samples;
    u32 loop_start;
    u32 loop_end;
    u32 loop_wait;
    int loop_times;
};

using controller_hid_handle_t = hid_device*;

struct RumbleData {
    std::string from_file;
    VIBMetadata metadata;
    std::shared_ptr<u8> data;
};

enum IRColor : u8 {
    IRGreyscale,
    IRNightVision,
    IRIronbow,
    IRInfrared,
    IRColorCount
};
/**
 * Enum values for the various resolutions of the IR Camera.
 * These are based on the normal orientation of the camera,
 * which is when the rail is facing upwards.
 */
enum IRResolution : u8 {
    IR_320x240 = 0b00000000,
    IR_160x120 = 0b1010000,
    IR_80x60 = 0b01100100,
    IR_40x30 = 0b01101001
};

enum IRCaptureMode : u8 {
    Off,
    Image,
    Video
};

struct IRCaptureStatus {
    float fps{};
    int frame_counter{};
    int last_frag_no{};
    float duration{};
    float noise_level{};
    int avg_intensity_percent{};
    int white_pixels_percent{};
    u16 exfilter{};
    u8 exf_int{};
    std::stringstream message_stream;
};

struct IRCaptureCTX {
    controller_hid_handle_t handle;
    u8& timming_byte; 
    ir_image_config& ir_cfg;
    u8& ir_max_frag_no;
    IRCaptureMode& capture_mode;
    IRCaptureStatus& capture_status;
};

struct Size2D {
    union {
        u16 x, width;
    };
    union {
        u16 y, height;
    };
};

#pragma pack(push, 1)
struct SPIColors {
    struct color_t {
        u8 r;
        u8 g;
        u8 b;
    };
    color_t body;
    color_t buttons;
    color_t left_grip;
    color_t right_grip;
};
#pragma pack(pop)

struct DumpSPICTX {
    bool& cancel_spi_dump;
    size_t& bytes_dumped;
    const char* file_name;
};

namespace ConHID {
    // Vendor ID
    const inline u16 VID = 0x57e;

    // Product IDs
    enum ProdID : u16 {
        NoCon = 0x0000,
        JoyConLeft = 0x2006,
        JoyConRight = 0x2007,
        ProCon = 0x2009,
        JoyConGrip = 0x200e,
    };
}


/**
 * controller handle and timming byte reference.
 */
struct CT {
    controller_hid_handle_t& handle;
    u8& timming_byte;
};
