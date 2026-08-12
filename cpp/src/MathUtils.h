#ifndef KIMHYUNHA_MATHUTILS_H
#define KIMHYUNHA_MATHUTILS_H

namespace kimhyunha {

class MathUtils {
public:
    static double degrees_to_radians(double degrees);
    static double radians_to_degrees(double radians);
    static double calculate_fps(double prev_time, double current_time);
    static double clamp(double value, double min, double max);
    static double lerp(double a, double b, double t);
};

}  // namespace kimhyunha

#endif  // KIMHYUNHA_MATHUTILS_H