#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_CARGO_IMPACT_ENABLED
#define AP_CARGO_IMPACT_ENABLED (HAL_PROGRAM_SIZE_LIMIT_KB > 1024)
#endif
