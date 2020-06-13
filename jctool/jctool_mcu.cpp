#include <cstring>

#include "jctool.h"
#include "jctool_mcu.hpp"
#include "jctool_helpers.hpp"

#ifdef WIN32
#include <cstdio>
#include <Windows.h>
#else
#include <stdio.h>
#include <unistd.h>
inline int Sleep(uint64_t ms){
    return usleep(ms*1000);
};
#include <math.h> // for sqrt
const auto min = [](auto a, auto b){
    return (a < b) ? a : b;
};
#endif

namespace MCU {
    u8 mcu_crc8_calc(u8 *buf, u8 size) {
        u8 crc8 = 0x0;

        for (int i = 0; i < size; ++i) {
            crc8 = mcu_crc8_table[(u8)(crc8 ^ buf[i])];
        }
        return crc8;
    }

    #ifndef __jctool_disable_legacy_ui__
    std::string get_sn(u32 offset, const u16 read_len) {
    #else
    std::string get_sn(CT& ct) {
        static const u32 offset = 0x6001;
        static const u16 read_len = 0xF;
        controller_hid_handle_t& handle = ct.handle;
        u8& timming_byte = ct.timming_byte;
    #endif
        int res;
        int error_reading = 0;
        u8 buf[49];
        std::string test = "";
        while (1) {
            memset(buf, 0, sizeof(buf));
            auto hdr = (brcm_hdr *)buf;
            auto pkt = (brcm_cmd_01 *)(hdr + 1);
            hdr->cmd = 1;
            hdr->timer = timming_byte & 0xF;
            timming_byte++;
            pkt->subcmd = 0x10;
            pkt->spi_data.offset = offset;
            pkt->spi_data.size = read_len;
            res = hid_write(handle, buf, sizeof(buf));

            int retries = 0;
            while (1) {
                res = hid_read_timeout(handle, buf, sizeof(buf), 64);
                if ((*(u16*)&buf[0xD] == 0x1090) && (*(uint32_t*)&buf[0xF] == offset))
                    goto check_result;

                retries++;
                if (retries > 8 || res == 0)
                    break;
            }
            error_reading++;
            if (error_reading > 20)
                return "Error!";
        }
        check_result:
        if (res >= 0x14 + read_len) {
            for (int i = 0; i < read_len; i++) {
                if (buf[0x14 + i] != 0x000) {
                    test += buf[0x14 + i];
                }else
                    test += "";
                }
        }
        else {
            return "Error!";
        }
        return test;
    }
    #ifndef __jctool_disable_legacy_ui__
    int get_spi_data(u32 offset, const u16 read_len, u8 *test_buf) {
    #else
    int get_spi_data(CT& ct, u32 offset, const u16 read_len, u8 * test_buf) {
        controller_hid_handle_t& handle = ct.handle;
        u8& timming_byte = ct.timming_byte;
    #endif
        int res;
        u8 buf[49];
        int error_reading = 0;
        while (1) {
            memset(buf, 0, sizeof(buf));
            auto hdr = (brcm_hdr *)buf;
            auto pkt = (brcm_cmd_01 *)(hdr + 1);
            hdr->cmd = 1;
            hdr->timer = timming_byte & 0xF;
            timming_byte++;
            pkt->subcmd = 0x10;
            pkt->spi_data.offset = offset;
            pkt->spi_data.size = read_len;
            res = hid_write(handle, buf, sizeof(buf));

            int retries = 0;
            while (1) {
                res = hid_read_timeout(handle, buf, sizeof(buf), 64);
                if ((*(u16*)&buf[0xD] == 0x1090) && (*(uint32_t*)&buf[0xF] == offset))
                    goto check_result;

                retries++;
                if (retries > 8 || res == 0)
                    break;
            }
            error_reading++;
            if (error_reading > 20)
                return 1;
        }
        check_result:
        if (res >= 0x14 + read_len) {
                for (int i = 0; i < read_len; i++) {
                    test_buf[i] = buf[0x14 + i];
                }
        }
        
        return 0;
    }

    #ifndef __jctool_disable_legacy_ui__
    int write_spi_data(u32 offset, const u16 write_len, u8* test_buf) {
    #else
    int write_spi_data(CT& ct, u32 offset, const u16 write_len, u8* test_buf) {
        controller_hid_handle_t& handle = ct.handle;
        u8& timming_byte = ct.timming_byte;
    #endif
        int res;
        u8 buf[49];
        int error_writing = 0;
        while (1) {
            memset(buf, 0, sizeof(buf));
            auto hdr = (brcm_hdr *)buf;
            auto pkt = (brcm_cmd_01 *)(hdr + 1);
            hdr->cmd = 1;
            hdr->timer = timming_byte & 0xF;
            timming_byte++;
            pkt->subcmd = 0x11;
            pkt->spi_data.offset = offset;
            pkt->spi_data.size = write_len;
            for (int i = 0; i < write_len; i++)
                buf[0x10 + i] = test_buf[i];

            res = hid_write(handle, buf, sizeof(buf));
            int retries = 0;
            while (1) {
                res = hid_read_timeout(handle, buf, sizeof(buf), 64);
                if (*(u16*)&buf[0xD] == 0x1180)
                    goto check_result;

                retries++;
                if (retries > 8 || res == 0)
                    break;
            }
            error_writing++;
            if (error_writing == 20)
                return 1;
        }
        check_result:
        return 0;
    }

    #ifndef __jctool_disable_legacy_ui__
    int get_device_info(u8* test_buf) {
    #else
    int get_device_info(CT& ct, u8* test_buf) {
        controller_hid_handle_t& handle = ct.handle;
        u8& timming_byte = ct.timming_byte;
    #endif
        int res;
        u8 buf[49];
        int error_reading = 0;
        while (1) {
            memset(buf, 0, sizeof(buf));
            auto hdr = (brcm_hdr *)buf;
            auto pkt = (brcm_cmd_01 *)(hdr + 1);
            hdr->cmd = 1;
            hdr->timer = timming_byte & 0xF;
            timming_byte++;
            pkt->subcmd = 0x02;
            res = hid_write(handle, buf, sizeof(buf));
            int retries = 0;
            while (1) {
                res = hid_read_timeout(handle, buf, sizeof(buf), 64);        
                if (*(u16*)&buf[0xD] == 0x0282)
                    goto check_result;

                retries++;
                if (retries > 8 || res == 0)
                    break;
            }
            error_reading++;
            if (error_reading > 20)
                break;
        }
        check_result:
        for (int i = 0; i < 0xA; i++) {
            test_buf[i] = buf[0xF + i];
        }

        return 0;
    }

    #ifndef __jctool_disable_legacy_ui__
    int get_battery(u8* test_buf) {
    #else
    int get_battery(CT& ct, u8* test_buf) {
        controller_hid_handle_t& handle = ct.handle;
        u8& timming_byte = ct.timming_byte;
    #endif
        int res;
        u8 buf[49];
        int error_reading = 0;
        while (1) {
            memset(buf, 0, sizeof(buf));
            auto hdr = (brcm_hdr *)buf;
            auto pkt = (brcm_cmd_01 *)(hdr + 1);
            hdr->cmd = 1;
            hdr->timer = timming_byte & 0xF;
            timming_byte++;
            pkt->subcmd = 0x50;
            res = hid_write(handle, buf, sizeof(buf));
            int retries = 0;
            while (1) {
                res = hid_read_timeout(handle, buf, sizeof(buf), 64);
                if (*(u16*)&buf[0xD] == 0x50D0)
                    goto check_result;

                retries++;
                if (retries > 8 || res == 0)
                    break;
            }
            error_reading++;
            if (error_reading > 20)
                break;
        }
        check_result:
        test_buf[0] = buf[0x2];
        test_buf[1] = buf[0xF];
        test_buf[2] = buf[0x10];

        return 0;
    }
    #ifndef __jctool_disable_legacy_ui__
    int get_temperature(u8* test_buf) {
    #else
    int get_temperature(CT& ct, u8* test_buf) {
        controller_hid_handle_t& handle = ct.handle;
        u8& timming_byte = ct.timming_byte;
    #endif
        int res;
        u8 buf[49];
        int error_reading = 0;
        bool imu_changed = false;

        while (1) {
            memset(buf, 0, sizeof(buf));
            auto hdr = (brcm_hdr *)buf;
            auto pkt = (brcm_cmd_01 *)(hdr + 1);
            hdr->cmd = 1;
            hdr->timer = timming_byte & 0xF;
            timming_byte++;
            pkt->subcmd = 0x43;
            pkt->subcmd_arg.arg1 = 0x10;
            pkt->subcmd_arg.arg2 = 0x01;
            res = hid_write(handle, buf, sizeof(buf));
            int retries = 0;
            while (1) {
                res = hid_read_timeout(handle, buf, sizeof(buf), 64);
                if (*(u16*)&buf[0xD] == 0x43C0)
                    goto check_result;

                retries++;
                if (retries > 8 || res == 0)
                    break;
            }
            error_reading++;
            if (error_reading > 20)
                break;
        }
        check_result:
        if ((buf[0x11] >> 4) == 0x0) {

            memset(buf, 0, sizeof(buf));
            auto hdr = (brcm_hdr *)buf;
            auto pkt = (brcm_cmd_01 *)(hdr + 1);
            hdr->cmd = 0x01;
            hdr->timer = timming_byte & 0xF;
            timming_byte++;
            pkt->subcmd = 0x40;
            pkt->subcmd_arg.arg1 = 0x01;
            res = hid_write(handle, buf, sizeof(buf));
            res = hid_read_timeout(handle, buf, 1, 64);

            imu_changed = true;

            // Let temperature sensor stabilize for a little bit.
            Sleep(64);
        }
        error_reading = 0;
        while (1) {
            memset(buf, 0, sizeof(buf));
            auto hdr = (brcm_hdr *)buf;
            auto pkt = (brcm_cmd_01 *)(hdr + 1);
            hdr->cmd = 1;
            hdr->timer = timming_byte & 0xF;
            timming_byte++;
            pkt->subcmd = 0x43;
            pkt->subcmd_arg.arg1 = 0x20;
            pkt->subcmd_arg.arg2 = 0x02;
            res = hid_write(handle, buf, sizeof(buf));
            int retries = 0;
            while (1) {
                res = hid_read_timeout(handle, buf, sizeof(buf), 64);
                if (*(u16*)&buf[0xD] == 0x43C0)
                    goto check_result2;

                retries++;
                if (retries > 8 || res == 0)
                    break;
            }
            error_reading++;
            if (error_reading > 20)
                break;
        }
        check_result2:
        test_buf[0] = buf[0x11];
        test_buf[1] = buf[0x12];

        if (imu_changed) {
            memset(buf, 0, sizeof(buf));
            auto hdr = (brcm_hdr *)buf;
            auto pkt = (brcm_cmd_01 *)(hdr + 1);
            hdr->cmd = 0x01;
            hdr->timer = timming_byte & 0xF;
            timming_byte++;
            pkt->subcmd = 0x40;
            pkt->subcmd_arg.arg1 = 0x00;
            res = hid_write(handle, buf, sizeof(buf));
            res = hid_read_timeout(handle, buf, 1, 64);
        }

        return 0;
    }

    #ifndef __jctool_disable_legacy_ui__
    int dump_spi(const char *dev_name) {
    #else
    int dump_spi(CT& ct, DumpSPICTX& dump_spi_ctx) {
        controller_hid_handle_t& handle = ct.handle;
        u8& timming_byte = ct.timming_byte;
        const char*& dev_name = dump_spi_ctx.file_name;
        bool& cancel_spi_dump = dump_spi_ctx.cancel_spi_dump;
    #endif
        int error_reading = 0;
        std::string file_dev_name = dev_name;
    #ifndef __jctool_disable_legacy_ui__
        String^ filename_sys = gcnew String(file_dev_name.c_str());
    #endif
        file_dev_name = "./" + file_dev_name;

        FILE *f;

    #ifdef WIN32
        errno_t err;
        if ((err = fopen_s(&f, file_dev_name.c_str(), "wb")) != 0) {
    #elif defined(__linux__)
        if ((f = fopen(file_dev_name.c_str(), "wb")) == nullptr) {
    #endif
    #ifndef __jctool_disable_legacy_ui__
            MessageBox::Show(L"Cannot open file " + filename_sys + L" for writing!\n\nError: " + err, L"Error opening file!", MessageBoxButtons::OK ,MessageBoxIcon::Exclamation);
    #endif
            
            return 1;
        }

        int res;
        u8 buf[49];
        
        u16 read_len = 0x1d;
        u32 offset = 0x0;
        while (offset < SPI_SIZE && !cancel_spi_dump) {
            error_reading = 0;
    #ifndef __jctool_disable_legacy_ui__
            std::stringstream offset_label;
            offset_label << std::fixed << std::setprecision(2) << std::setfill(' ') << offset/1024.0f;
            offset_label << "KB of 512KB";
            FormJoy::myform1->label_progress->Text = gcnew String(offset_label.str().c_str());
            Application::DoEvents();
    #endif

            while(1) {
                memset(buf, 0, sizeof(buf));
                auto hdr = (brcm_hdr *)buf;
                auto pkt = (brcm_cmd_01 *)(hdr + 1);
                hdr->cmd = 1;
                hdr->timer = timming_byte & 0xF;
                timming_byte++;
                pkt->subcmd = 0x10;
                pkt->spi_data.offset = offset;
                pkt->spi_data.size = read_len;
                res = hid_write(handle, buf, sizeof(buf));
                int retries = 0;
                while (1) {
                    res = hid_read_timeout(handle, buf, sizeof(buf), 64);
                    if ((*(u16*)&buf[0xD] == 0x1090) && (*(uint32_t*)&buf[0xF] == offset))
                        goto check_result;

                    retries++;
                    if (retries > 8 || res == 0)
                        break;
                }
                if (retries > 8)
                    error_reading++;
                if (error_reading > 10) {
                    fclose(f);
                    return 1;
                }     
            }
            check_result:
            fwrite(buf + 0x14, read_len, 1, f);
            offset += read_len;
            if (offset == 0x7FFE6)
                read_len = 0x1A;
    #ifdef __jctool_disable_legacy_ui__
            dump_spi_ctx.bytes_dumped = offset;
    #endif
        }
        fclose(f);

        return 0;
    }

    struct SPIRestoreCTX {
        CT* ct;
        u8 backup_spi[SPI_SIZE];
    };

    bool validate_spi(SPIRestoreCTX* restore_ctx, ConHID::ProdID prod_id){
        struct ValidationMagic {
            enum Con {
                None,
                JoyL,
                JoyR,
                Pro
            };

            u8 magic[36] = { 
                0x01, 0x08, 0x00, 0xF0, 0x00,
                0x00, 0x62, 0x08, 0xC0, 0x5D,
                0x89, 0xFD, 0x04, 0x00, 0xFF,
                0xFF, 0xFF, 0xFF, 0x40, 0x06,
                None, 0xA0, 0x0A, 0xFB, 0x00,
                0x00, 0x02, 0x0D, 0xAA, 0x55,
                0xF0, 0x0F, 0x68, 0xE5, 0x97, 0xD2
            };
            
            ValidationMagic(u8 con_type)
            { magic[20] = con_type; }

            /**
             * Return true if the spi is valid.
             */
            bool validateSPI(u8* backup_spi) {
                // Check if the backup spi is valid for the controller.
                if(backup_spi[0x6012] != magic[20]
                || backup_spi[0x6012 + 1] != magic[21])
                    return false;
                
                for(int i=0; i < 20; i++)
                    if(magic[i] != backup_spi[i])
                        return false;
                
                bool ota_exists{ true };
                for(int i=28; i < 36; i++)
                    if(magic[i] != backup_spi[0x1FF4 + i - 22]){
                        ota_exists = false;
                        break;
                    }
                
                if(ota_exists){
                    for(int i=22; i < 28; i++) {
                        if(magic[i] != backup_spi[0x10000 + i - 22]
                        && i != 23)
                            return false;
                        if(magic[i] != backup_spi[0x28000 + i - 22]
                        && i != 23)
                            return false;
                    }
                } else {
                    for(int i=22; i < 28; i++)
                        if(magic[i] != backup_spi[0x10000 + i - 22]
                        && i != 23)
                            return false;
                }
                return true;
            }
        };
        u8 con_type;
        switch (prod_id)
        {
        case ConHID::JoyConLeft:
            con_type = 1;
            break;
        case ConHID::JoyConRight:
            con_type = 2;
            break;
        case ConHID::ProCon:
            con_type = 3;
            break;
        default:
            return false;
        }
        ValidationMagic magic = ValidationMagic(con_type);

        return magic.validateSPI(restore_ctx->backup_spi);
    }

    bool spi_mac_check(SPIRestoreCTX* restore_ctx){
        bool mac_check = true;
        u8 mac_addr[10];
        memset(mac_addr, 0, sizeof(mac_addr));
        get_device_info(*restore_ctx->ct, mac_addr);

        for(int i=4; i < 10; i++)
            if(mac_addr[i] != restore_ctx->backup_spi[0x1A - i + 4])
                mac_check = false;
        return mac_check;
    }

#define RESTORE_NOTIF(type, code) (type | (code << 16))
#define BREAK_OR_CONTINUE_ON_RESTORE_RES_err(restore_notif_code_err) \
        if(restore_res != 0){\
            cb(RESTORE_NOTIF(RestoreProgressError, restore_notif_code_err), opaque_notify);\
            break;\
        }
#define NOTIFY_RESTORE_POINT(restore_notif_code) \
        cb(RESTORE_NOTIF(RestoreProgressUpdate, restore_notif_code), opaque_notify);
            

    void spi_restore(CT* ct, ConHID::ProdID prod_id, const char* spi_filepath, restore_notify_cb cb, restore_progress_cb cb_prog, void* opaque_notify, void* opaque_prog) {
        SPIRestoreCTX restore_ctx;
        memset(&restore_ctx, 0, sizeof(restore_ctx));
        restore_ctx.ct = ct;

        FILE* spi_file = fopen(spi_filepath, "rb");
        if(!spi_file){
            // Unable to open the file for reading.
            cb(RESTORE_NOTIF(RestoreError, 1), opaque_notify);
            return; 
        }

        fseek(spi_file, 0, SEEK_END);
        size_t file_size = ftell(spi_file);
        rewind(spi_file);
        if(file_size != SPI_SIZE){
            // The file size is not that of the spi size.
            cb(RESTORE_NOTIF(RestoreError, 2), opaque_notify);
            return;
        }

        size_t bytes_read = fread(restore_ctx.backup_spi, sizeof(u8), SPI_SIZE, spi_file);
        fclose(spi_file);
        if(bytes_read != SPI_SIZE){
            // The bytes read from the file is not that of the spi size.
            cb(RESTORE_NOTIF(RestoreError, 3), opaque_notify);
            return;
        }
        
        if(!validate_spi(&restore_ctx, prod_id)){
            // The spi was not valid for the controller type.
            cb(RESTORE_NOTIF(RestoreError, 4), opaque_notify);
            return;
        }

        // Request the restoration mode.
        RestoreMode mode = (RestoreMode) cb(RestoreSelectMode, opaque_notify);
        bool is_same_mac = spi_mac_check(&restore_ctx);
        bool valid_restore_mode;
        const int RETRIES_MAX = 5;
        int retries = 0;
        while(true) {
            valid_restore_mode = (mode > Cancel && mode < RestoreModeMax)
                                 && !(mode == Full && !is_same_mac);
            if(mode == Cancel){
                // Restoration canceled.
                cb(RESTORE_NOTIF(RestoreError, 5), opaque_notify);
                return;
            } else
            if(!valid_restore_mode){
                if (retries == RETRIES_MAX){
                    // Too many tries for an invalid restoration mode.
                    cb(RESTORE_NOTIF(RestoreError, 7), opaque_notify);
                    return;
                }
                // The restore mode method must be valid.
                // Also, full restore is disabled if the backup does not match the device mac.
                // Ask again and hope for a different but compatible restore mode or -1 to cancel restore.
                mode = (RestoreMode) cb(RESTORE_NOTIF(RestoreSelectMode, retries), opaque_notify);
                retries++;
            } else
                // The mode is a different restoration method that isn't Cancel or an invalid mode.
                break; // Exit the loop and perform the restoration method.
        }

        int restore_res = 0;
        cb(RESTORE_NOTIF(RestoreProgressUpdate, RestoreBegin), opaque_notify);
        cb_prog(0.0f, opaque_prog);
        switch(mode){
            case DeviceColors:{
                u8 backup_color[12];
                memcpy(backup_color, &restore_ctx.backup_spi[0x6050], sizeof(backup_color));
                restore_res =  write_spi_data(*restore_ctx.ct, 0x6050, sizeof(backup_color), backup_color);
                cb_prog(1.0f, opaque_prog);
            }
            break;
            case SerialNumber:{
                u8 sn[0x10];
                memcpy(sn, &restore_ctx.backup_spi[0x6000], sizeof(sn));
                NOTIFY_RESTORE_POINT(RestoreSN)
                restore_res = write_spi_data(*restore_ctx.ct, 0x6000, sizeof(sn), sn);
                BREAK_OR_CONTINUE_ON_RESTORE_RES_err(RestoreSN)
                cb_prog(1.0f, opaque_prog);
            }
            break;
            case UserCalibration:{
                u8 l_stick[0xB];
                u8 r_stick[0xB];
                u8 sensor[0x1A];

                const off_t r_stick_off = 0x8010 + sizeof(l_stick);
                const off_t sensor_off = r_stick_off + sizeof(r_stick);
                size_t total_size = sizeof(sensor);
                size_t prog = 0;
                switch(prod_id){
                    case ConHID::ProCon:
                        total_size += sizeof(l_stick) * 2;
                    break;
                    default:
                        total_size += sizeof(l_stick);
                    break;
                }

                memcpy(l_stick, &restore_ctx.backup_spi[0x8010], sizeof(l_stick));
                memcpy(r_stick, &restore_ctx.backup_spi[r_stick_off], sizeof(r_stick));
                memcpy(sensor, &restore_ctx.backup_spi[sensor_off], sizeof(sensor));

                if(prod_id != ConHID::JoyConRight) {
                    NOTIFY_RESTORE_POINT(RestoreLStickCal)
                    restore_res = write_spi_data(*restore_ctx.ct, 0x8010, sizeof(l_stick), l_stick);
                    BREAK_OR_CONTINUE_ON_RESTORE_RES_err(RestoreLStickCal)
                    prog += sizeof(l_stick);
                    cb_prog((float) prog / total_size, opaque_prog);
                }

                if(prod_id != ConHID::JoyConLeft) {
                    NOTIFY_RESTORE_POINT(RestoreRStickCal)
                    restore_res = write_spi_data(*restore_ctx.ct, r_stick_off, sizeof(r_stick), r_stick);
                    BREAK_OR_CONTINUE_ON_RESTORE_RES_err(RestoreRStickCal)
                    prog += sizeof(r_stick);
                    cb_prog((float) prog / total_size, opaque_prog);
                }
                NOTIFY_RESTORE_POINT(RestoreSensorCal)
                restore_res = write_spi_data(*restore_ctx.ct, sensor_off, sizeof(sensor), sensor);
                BREAK_OR_CONTINUE_ON_RESTORE_RES_err(RestoreSensorCal)
                prog += sizeof(sensor);
                cb_prog((float) prog / total_size, opaque_prog);
            }
            break;
            case OEMReset:{ // Unimplemented.
                restore_res = -1;
            }
            break;
            case Full:{
                const size_t FACTORY_CFG_SIZE = 0x1000;
                const size_t USER_CAL_SIZE = 0x1000;
                const size_t SN_SIZE = 0x10;
                const size_t REINIT_SIZE = 0x7 * 3;
                const size_t TOTAL_SIZE = FACTORY_CFG_SIZE + USER_CAL_SIZE + SN_SIZE + REINIT_SIZE;

                u8 full_restore_buf[0x10];
                u8 sn_backup_erase[0x10];
                NOTIFY_RESTORE_POINT(RestoreFactoryCfg)
                // Factory config 0x6000
                for(int i=0; i < FACTORY_CFG_SIZE; i += 0x10){
                    memcpy(full_restore_buf, &restore_ctx.backup_spi[0x6000 + i], sizeof(full_restore_buf));
                    restore_res = write_spi_data(*restore_ctx.ct, 0x6000 + i, sizeof(full_restore_buf), full_restore_buf);
                    if(restore_res != 0)
                        break;
                    cb_prog((float) i / TOTAL_SIZE, opaque_prog);
                }

                BREAK_OR_CONTINUE_ON_RESTORE_RES_err(RestoreFactoryCfg)
                NOTIFY_RESTORE_POINT(RestoreUserCal)
                // User calibration 0x8000
                for(int i=0; i < USER_CAL_SIZE; i += 0x10){
                    memcpy(full_restore_buf, &restore_ctx.backup_spi[0x8000 + i], sizeof(full_restore_buf));
                    restore_res = write_spi_data(*restore_ctx.ct, 0x8000 + i, sizeof(full_restore_buf), full_restore_buf);
                    if(restore_res != 0)
                        break;
                    cb_prog((float) (i + FACTORY_CFG_SIZE) / TOTAL_SIZE, opaque_prog);
                }
                BREAK_OR_CONTINUE_ON_RESTORE_RES_err(RestoreUserCal)
                NOTIFY_RESTORE_POINT(RestoreSN)
                // Erase S/N backup storage.
                restore_res = write_spi_data(*restore_ctx.ct, 0xF000, sizeof(sn_backup_erase), sn_backup_erase);
                BREAK_OR_CONTINUE_ON_RESTORE_RES_err(RestoreSN)
                cb_prog((float) (FACTORY_CFG_SIZE + USER_CAL_SIZE + SN_SIZE) / TOTAL_SIZE, opaque_prog);

                NOTIFY_RESTORE_POINT(RestoreReinit)
                // Set shipment
                unsigned char custom_cmd[7];
                memset(custom_cmd, 0, 7);
                custom_cmd[0] = 0x01;
                custom_cmd[5] = 0x08;
                custom_cmd[6] = 0x01;
                restore_res = send_custom_command(*restore_ctx.ct, custom_cmd);
                BREAK_OR_CONTINUE_ON_RESTORE_RES_err(RestoreReinit)
                // Clear pairing info
                memset(custom_cmd, 0, 7);
                custom_cmd[0] = 0x01;
                custom_cmd[5] = 0x07;
                restore_res = send_custom_command(*restore_ctx.ct, custom_cmd);
                BREAK_OR_CONTINUE_ON_RESTORE_RES_err(RestoreReinit)
                // Reboot controller and go into pairing mode
                memset(custom_cmd, 0, 7);
                custom_cmd[0] = 0x01;
                custom_cmd[5] = 0x06;
                custom_cmd[6] = 0x02;
                restore_res = send_custom_command(*restore_ctx.ct, custom_cmd);
                BREAK_OR_CONTINUE_ON_RESTORE_RES_err(RestoreReinit)

                cb_prog((float) (FACTORY_CFG_SIZE + USER_CAL_SIZE + SN_SIZE + REINIT_SIZE) / TOTAL_SIZE, opaque_prog);
            }
            break;
        }

        if(restore_res != 0)
            cb(RESTORE_NOTIF(RestoreProgressError, RestoreEnd), opaque_notify);
        else
            cb(RESTORE_NOTIF(RestoreProgressUpdate, RestoreEnd), opaque_notify);
    }

    #ifndef __jctool_disable_legacy_ui__
    int send_custom_command(u8* arg) {
    #else
    int send_custom_command(CT& ct, u8* arg){
        controller_hid_handle_t& handle = ct.handle;
        u8& timming_byte = ct.timming_byte;
    #endif
        int res_write;
        int res;
        int byte_seperator = 1;
    #ifndef __jctool_disable_legacy_ui__
        String^ input_report_cmd;
        String^ input_report_sys;
        String^ output_report_sys;
    #else
        std::ostringstream input_report_cmd;
        std::ostringstream input_report_sys;
        std::ostringstream output_report_sys;
    #endif
        u8 buf_cmd[49];
        u8 buf_reply[0x170];
        memset(buf_cmd, 0, sizeof(buf_cmd));
        memset(buf_reply, 0, sizeof(buf_reply));

        buf_cmd[0] = arg[0]; // cmd
        buf_cmd[1] = timming_byte & 0xF;
        timming_byte++;
        // Vibration pattern
        buf_cmd[2] = buf_cmd[6] = arg[1];
        buf_cmd[3] = buf_cmd[7] = arg[2];
        buf_cmd[4] = buf_cmd[8] = arg[3];
        buf_cmd[5] = buf_cmd[9] = arg[4];

        buf_cmd[10] = arg[5]; // subcmd

        // subcmd x21 crc byte
        if (arg[5] == 0x21)
            arg[43] = MCU::mcu_crc8_calc(arg + 7, 36);
    #ifndef __jctool_disable_legacy_ui__
        output_report_sys = String::Format(L"Cmd:  {0:X2}   Subcmd: {1:X2}\r\n", buf_cmd[0], buf_cmd[10]);
    #else
        // TODO: Implement else
        //output_report_sys << "Cmd: " << std::setbase(hex) << buf_cmd[0]
    #endif
        if (buf_cmd[0] == 0x01 || buf_cmd[0] == 0x10 || buf_cmd[0] == 0x11) {
            for (int i = 6; i < 44; i++) {
                buf_cmd[5 + i] = arg[i];
    #ifndef __jctool_disable_legacy_ui__
                output_report_sys += String::Format(L"{0:X2} ", buf_cmd[5 + i]);
                if (byte_seperator == 4)
                    output_report_sys += L" ";
    #else
                // TODO: Implement else
    #endif
                if (byte_seperator == 8) {
                    byte_seperator = 0;
    #ifndef __jctool_disable_legacy_ui__
                    output_report_sys += L"\r\n";
    #else
                    // TODO: Implement else
    #endif
                }
                byte_seperator++;
            }
        }
        //Use subcmd after command
        else {
            for (int i = 6; i < 44; i++) {
                buf_cmd[i - 5] = arg[i];
    #ifndef __jctool_disable_legacy_ui__
                output_report_sys += String::Format(L"{0:X2} ", buf_cmd[i - 5]);
                if (byte_seperator == 4)
                    output_report_sys += L" ";
    #else
                // TODO: Implement else
    #endif
                if (byte_seperator == 8) {
                    byte_seperator = 0;
    #ifndef __jctool_disable_legacy_ui__
                    output_report_sys += L"\r\n";
    #else
                    // TODO: Implement else
    #endif
                }
                byte_seperator++;
            }
        }
    #ifndef __jctool_disable_legacy_ui__
        FormJoy::myform1->textBoxDbg_sent->Text = output_report_sys;
    #else
        // TODO: Implement else
    #endif

        //Packet size header + subcommand and uint8 argument
        res_write = hid_write(handle, buf_cmd, sizeof(buf_cmd));
    #ifndef __jctool_disable_legacy_ui__
        if (res_write < 0)
            input_report_sys += L"hid_write failed!\r\n\r\n";
    #else
        // TODO: Implement else
    #endif
        int retries = 0;
        while (1) {
            res = hid_read_timeout(handle, buf_reply, sizeof(buf_reply), 64);

            if (res > 0) {
                if (arg[0] == 0x01 && buf_reply[0] == 0x21)
                    break;
                else if (arg[0] != 0x01)
                    break;
            }

            retries++;
            if (retries == 20)
                break;
        }
        byte_seperator = 1;
        if (res > 12) {
            if (buf_reply[0] == 0x21 || buf_reply[0] == 0x30 || buf_reply[0] == 0x33 || buf_reply[0] == 0x31 || buf_reply[0] == 0x3F) {
    #ifndef __jctool_disable_legacy_ui__
                input_report_cmd += String::Format(L"\r\nInput report: 0x{0:X2}\r\n", buf_reply[0]);
                input_report_sys += String::Format(L"Subcmd Reply:\r\n", buf_reply[0]);
    #else
                // TODO: Implement else
    #endif
                int len = 49;
                if (buf_reply[0] == 0x33 || buf_reply[0] == 0x31)
                    len = 362;
                for (int i = 1; i < 13; i++) {
    #ifndef __jctool_disable_legacy_ui__
                    input_report_cmd += String::Format(L"{0:X2} ", buf_reply[i]);
                    if (byte_seperator == 4)
                        input_report_cmd += L" ";
    #else
                    // TODO: Implement else
    #endif
                    if (byte_seperator == 8) {
                        byte_seperator = 0;
    #ifndef __jctool_disable_legacy_ui__
                        input_report_cmd += L"\r\n";
    #else
                        // TODO: Implement else
    #endif
                    }
                    byte_seperator++;
                }
                byte_seperator = 1;
                for (int i = 13; i < len; i++) {
    #ifndef __jctool_disable_legacy_ui__
                    input_report_sys += String::Format(L"{0:X2} ", buf_reply[i]);
                    if (byte_seperator == 4)
                        input_report_sys += L" ";
    #else
                    // TODO: Implement else
    #endif
                    if (byte_seperator == 8) {
                        byte_seperator = 0;
    #ifndef __jctool_disable_legacy_ui__
                        input_report_sys += L"\r\n";
    #else
                        // TODO: Implement else
    #endif
                    }
                    byte_seperator++;
                }
                int crc_check_ok = 0;
                if (arg[5] == 0x21) {
                    crc_check_ok = (buf_reply[48] == MCU::mcu_crc8_calc(buf_reply + 0xF, 33));
    #ifndef __jctool_disable_legacy_ui__
                    if (crc_check_ok)
                        input_report_sys += L"(CRC OK)";
                    else
                        input_report_sys += L"(Wrong CRC)";
    #else
                    // TODO: Implement else
    #endif
                }
            }
            else {
    #ifndef __jctool_disable_legacy_ui__
                input_report_sys += String::Format(L"ID: {0:X2} Subcmd reply:\r\n", buf_reply[0]);
                for (int i = 13; i < res; i++) {
                    input_report_sys += String::Format(L"{0:X2} ", buf_reply[i]);
                    if (byte_seperator == 4)
                        input_report_sys += L" ";
                    if (byte_seperator == 8) {
                        byte_seperator = 0;
                        input_report_sys += L"\r\n";
                    }
                    byte_seperator++;
                }
    #else
                // TODO: Implement else
    #endif
            }
        }
        else if (res > 0 && res <= 12) {
    #ifndef __jctool_disable_legacy_ui__
            for (int i = 0; i < res; i++)
                input_report_sys += String::Format(L"{0:X2} ", buf_reply[i]);
    #else
            // TODO: Implement else
    #endif
        }
        else {
    #ifndef __jctool_disable_legacy_ui__
            input_report_sys += L"No reply";
    #else
            // TODO: Implement else
    #endif
        }
    #ifndef __jctool_disable_legacy_ui__
        FormJoy::myform1->textBoxDbg_reply->Text = input_report_sys;
        FormJoy::myform1->textBoxDbg_reply_cmd->Text = input_report_cmd;
    #else
        // TODO: Implement else
    #endif

        return 0;
    }


    /**
     * ===========================================================================
     * Below is some code extracted from the orginal UI framework source, as well
     * as other bits and pieces originating from the early Joy-Con Toolkit from
     * CTCaer, and is now conveniently placed in callable functions.
     * Goal: Eliminate dependency to the original UI framework so that useful code
     * that was once only accessible through that same framework is now accessible
     * through an API.
     * ===========================================================================
     */

    BatteryData parseBatteryData(const unsigned char* batt_data) {
        int batt_percent = 0;
        int batt = ((u8)batt_data[0] & 0xF0) >> 4;

        // Calculate aproximate battery percent from regulated voltage
        u16 batt_volt = (u8)batt_data[1] + ((u8)batt_data[2] << 8);
        if (batt_volt < 0x560)
            batt_percent = 1;
        else if (batt_volt > 0x55F && batt_volt < 0x5A0) {
            batt_percent = static_cast<int>(((batt_volt - 0x60) & 0xFF) / 7.0f) + 1;
        }
        else if (batt_volt > 0x59F && batt_volt < 0x5E0) {
            batt_percent = static_cast<int>(((batt_volt - 0xA0) & 0xFF) / 2.625f) + 11;
        }
        else if (batt_volt > 0x5DF && batt_volt < 0x618) {
            batt_percent = static_cast<int>((batt_volt - 0x5E0) / 1.8965f) + 36;
        }
        else if (batt_volt > 0x617 && batt_volt < 0x658) {
            batt_percent = static_cast<int>(((batt_volt - 0x18) & 0xFF) / 1.8529f) + 66;
        }
        else if (batt_volt > 0x657)
            batt_percent = 100;

        return {batt_percent, batt, (batt_volt * 2.5f) / 1000};
    }

    TemperatureData parseTemperatureData(const unsigned char* temp_data){
        // Convert reading to Celsius according to datasheet
        float celsius = 25.0f + uint16_to_int16(temp_data[1] << 8 | temp_data[0]) * 0.0625f;
        return {celsius, celsius*1.8f + 32};
    }
    SPIColors get_spi_colors(CT& ct){
        unsigned char spi_colors[12];
        memset(spi_colors, 0, 12);

        int res = get_spi_data(ct, 0x6050, 12, spi_colors);

        SPIColors colors;
        memcpy(&colors, spi_colors, 12);
        return colors;
    }

    int write_spi_colors(CT& ct, const SPIColors& colors){
        unsigned char spi_colors[12];
        memcpy(spi_colors, &colors.body, sizeof(colors));

        return write_spi_data(ct, 0x6050, 12, spi_colors);
    }

    std::string ir_sensorErrorToString(int errno_ir_sensor){
        // Get error
        switch (errno_ir_sensor) {
        case 1:
            return "1ID31";
        case 2:
            return "2MCUON";
        case 3:
            return "3MCUONBUSY";
        case 4:
            return "4MCUMODESET";
        case 5:
            return "5MCUSETBUSY";
        case 6:
            return "6IRMODESET";
        case 7:
            return "7IRSETBUSY";
        case 8:
            return "8IRCFG";
        case 9:
            return "9IRFCFG";
        default:
            return "UNDEFINED_ERR";
        }
    }

    uint8_t mcu_crc8_table[] = {
        0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15, 0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
        0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65, 0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
        0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5, 0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
        0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85, 0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
        0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2, 0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
        0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2, 0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
        0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32, 0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
        0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42, 0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
        0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C, 0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
        0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC, 0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
        0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C, 0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
        0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C, 0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
        0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B, 0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
        0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B, 0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
        0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB, 0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
        0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB, 0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
    };


}