#include "Vector3D.h"

namespace kimhyunha {

Vector3D::Vector3D(double x, double y, double z) : Point3D(x, y, z) {}

double Vector3D::magnitude() const {
    return std::sqrt(x() * x() + y() * y() + z() * z());
}

Vector3D Vector3D::normalized() const {
    const double mag = magnitude();
    if (mag == 0.0) {
        return Vector3D(0.0, 0.0, 0.0);
    }
    return *this / mag;
}

double Vector3D::dot(const Vector3D& other) const {
    return x() * other.x() + y() * other.y() + z() * other.z();
}

Vector3D Vector3D::cross(const Vector3D& other) const {
    return Vector3D(
        y() * other.z() - z() * other.y(),
        z() * other.x() - x() * other.z(),
        x() * other.y() - y() * other.x());
}

double Vector3D::distance_to(const Vector3D& other) const {
    return (*this - other).magnitude();
}

Vector3D Vector3D::operator+(const Vector3D& other) const {
    return Vector3D(x() + other.x(), y() + other.y(), z() + other.z());
}

Vector3D Vector3D::operator-(const Vector3D& other) const {
    return Vector3D(x() - other.x(), y() - other.y(), z() - other.z());
}

Vector3D Vector3D::operator-() const {
    return Vector3D(-x(), -y(), -z());
}

Vector3D Vector3D::operator*(double scalar) const {
    return Vector3D(x() * scalar, y() * scalar, z() * scalar);
}

Vector3D Vector3D::operator/(double scalar) const {
    return Vector3D(x() / scalar, y() / scalar, z() / scalar);
}

Vector3D operator*(double scalar, const Vector3D& v) {
    return v * scalar;
}

}  // namespace kimhyunha
