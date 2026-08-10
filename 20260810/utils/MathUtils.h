#ifndef KIMHYUNHA_MATHUTILS_H
#define KIMHYUNHA_MATHUTILS_H

#include <algorithm>

class MathUtils {
public:
    template <typename T>
    static T clamp(T value, T lo, T hi) {
        return std::max(lo, std::min(value, hi));
    }
};

#endif // KIMHYUNHA_MATHUTILS_H
