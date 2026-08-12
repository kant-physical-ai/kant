#include "MathUtils.h"

#include <cmath>

namespace kimhyunha {

double MathUtils::degrees_to_radians(double degrees) {
    return degrees * (M_PI / 180.0);
}

double MathUtils::radians_to_degrees(double radians) {
    return radians * (180.0 / M_PI);
}

double MathUtils::calculate_fps(double prev_time, double current_time) {
    double delta = current_time - prev_time;
    if (delta <= 0.0) {
        return 0.0;
    }
    return 1.0 / delta;
}

double MathUtils::clamp(double value, double min, double max) {
    if (value < min) {
        return min;
    }
    if (value > max) {
        return max;
    }
    return value;
}

double MathUtils::lerp(double a, double b, double t) {
    return a + (b - a) * t;
}

}  // namespace kimhyunha