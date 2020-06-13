#pragma once
#include "jctool_types.h"

namespace MCU {
    enum RestoreMode {
        Cancel = -1,
        DeviceColors = 0,
        SerialNumber,
        UserCalibration,
        OEMReset,
        Full,
        RestoreModeMax
    };

    enum RestoreNotifType : uint16_t {
        RestoreUpdate = 0,
        RestoreError = 1 << 15,
        RestoreSelectMode = 1,
        RestoreProgressUpdate = RestoreUpdate | (1 << 1),
        RestoreProgressError = RestoreError | RestoreProgressUpdate
    };
    
    const unsigned short RestoreNotifTypeMask = ~uint16_t();
    const unsigned int RestoreNotifCodeMask = RestoreNotifTypeMask << 16;

    enum RestoreNotifCode: short {
        RestoreBegin,
        RestoreFactoryCfg,
        RestoreUserCal,
        RestoreReinit,
        RestoreColors,
        RestoreSN,
        RestoreLStickCal,
        RestoreRStickCal,
        RestoreSensorCal,
        RestoreEnd
    };

    /**
     * Composed of RestoreNotifType | (RestoreNotifCode << 16)
     */
    typedef int RestoreNotif;

    /**
     * This should be made to return -1 when the user wants to cancel an spi restoration.
     */
    typedef int (*restore_notify_cb)(int notif, void* opaque);
    /**
     * Progress is within the range 0.0f-1.0f.
     */
    typedef void (*restore_progress_cb)(float progress, void* opaque);

    // crc-8-ccitt / polynomial 0x07 look up table
    extern uint8_t mcu_crc8_table[];
    inline const size_t SPI_SIZE = 0x80000;

    u8 mcu_crc8_calc(u8 *buf, u8 size);
    std::string get_sn(CT& ct);
    int get_spi_data(CT& ct, u32 offset, const u16 read_len, u8 *test_buf);
    int write_spi_data(CT& ct, u32 offset, const u16 write_len, u8* test_buf);
    int write_spi_colors(CT& ct, const SPIColors& colors);
    int get_device_info(CT& ct, u8* test_buf);
    int get_battery(CT& ct, u8* test_buf);
    int get_temperature(CT& ct, u8* test_buf);
    int dump_spi(CT& ct, DumpSPICTX& dump_spi_ctx);
    SPIColors get_spi_colors(CT& ct);
    int send_custom_command(CT& ct, u8* arg);

    void spi_restore(CT* ct, ConHID::ProdID prod_id, const char* spi_filepath, restore_notify_cb cb, restore_progress_cb cb_prog, void* opaque_notify, void* opaque_prog);
}