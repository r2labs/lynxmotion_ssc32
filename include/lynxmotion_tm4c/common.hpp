#ifndef __LYNXMOTION_TM4C_COMMON_HPP__
#define __LYNXMOTION_TM4C_COMMON_HPP__

float lerp(float x, float x_min, float x_max, float y_min, float y_max) {
    if (x > x_max) { return y_max; }
    else if (x < x_min) { return y_min; }
    return y_min + (y_max - y_min)*((x - x_min)/(x_max - x_min));
}

#endif  /* __LYNXMOTION_TM4C_COMMON_HPP__ */
