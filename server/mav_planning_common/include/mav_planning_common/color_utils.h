#ifndef MAV_PLANNING_COMMON_COLOR_UTILS_H_
#define MAV_PLANNING_COMMON_COLOR_UTILS_H_

namespace mav_planning {

// Maps from a percent to an RGB value on a rainbow scale using math magic.
typedef struct ColorRGBA{
    float r;
    float g;
    float b;
    float a;
} ColorRGBA;

ColorRGBA percentToRainbowColor(double h);

}  // namespace mav_planning

#endif  // MAV_PLANNING_COMMON_COLOR_UTILS_H_
