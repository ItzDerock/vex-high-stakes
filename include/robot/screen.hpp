#pragma once
#include "liblvgl/core/lv_obj.h"
#include "pros-mpeg/mpeg.hpp"
namespace screen {

extern lv_obj_t *auton_selector_screen;

void initAutonSelector(MPEGPlayer *gif);

} // namespace screen
