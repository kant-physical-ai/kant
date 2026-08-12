#include "Volume3D.h"

namespace kimhyunha {

Volume3D::Volume3D(double width, double height, double depth)
    : width_(width), height_(height), depth_(depth) {}

double Volume3D::width() const { return width_; }
double Volume3D::height() const { return height_; }
double Volume3D::depth() const { return depth_; }
double Volume3D::volume() const { return width_ * height_ * depth_; }

}  // namespace kimhyunha