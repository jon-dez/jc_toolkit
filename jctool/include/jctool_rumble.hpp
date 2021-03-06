#pragma once
#include "jctool_types.h"

namespace Rumble{
    int send_rumble(CT& ct, ConHID::ProdID con_type);
    int play_tune(CT& ct, int tune_no);
    int play_hd_rumble_file(CT& ct, const RumbleData& rumble_data);
}
