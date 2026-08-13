#ifndef KIMHYUNHA_VECTOR3D_H
#define KIMHYUNHA_VECTOR3D_H

#include <cmath>

#include "Point3D.h"

namespace kimhyunha {

class Vector3D : public Point3D {
public:
    Vector3D(double x = 0.0, double y = 0.0, double z = 0.0);

    double magnitude() const;
    Vector3D normalized() const;
    double dot(const Vector3D& other) const;
    Vector3D cross(const Vector3D& other) const;
    double distance_to(const Vector3D& other) const;

    Vector3D operator+(const Vector3D& other) const;
    Vector3D operator-(const Vector3D& other) const;
    Vector3D operator-() const;
    Vector3D operator*(double scalar) const;
    Vector3D operator/(double scalar) const;

    friend Vector3D operator*(double scalar, const Vector3D& v);
};

}  // namespace kimhyunha

#endif  // KIMHYUNHA_VECTOR3D_H
