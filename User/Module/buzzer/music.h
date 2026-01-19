//
// Created by An on 2025/12/22.
//

#ifndef RM2026_EGADAPTER_MC02_BASE_MUSIC_H
#define RM2026_EGADAPTER_MC02_BASE_MUSIC_H

#include "buzzer_on_board.h"

namespace music{
    // 音符（频率）
    uint16_t melody[] = {
        523, 523, 587, 587, 659, 659, 587,
        523, 523, 494, 494, 440, 440, 392
    };

    // 时长（毫秒）
    uint16_t duration[] = {
        400, 400, 400, 400, 400, 400, 800,
        400, 400, 400, 400, 400, 400, 800
    };

}

#endif // RM2026_EGADAPTER_MC02_BASE_MUSIC_H
