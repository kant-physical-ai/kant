#include "Point3D.h"

namespace kimhyunha {

Point3D::Point3D(double x, double y, double z) : x_(x), y_(y), z_(z) {}

double Point3D::x() const { return x_; }
double Point3D::y() const { return y_; }
double Point3D::z() const { return z_; }

double Point3D::as_tuple(double out[3]) const {
    out[0] = x_;
    out[1] = y_;
    out[2] = z_;
    return out[0] + out[1] + out[2];
}

}  // namespace kimhyunha