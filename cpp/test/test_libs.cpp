#include <cmath>
#include <gtest/gtest.h>

#include "MathUtils.h"
#include "Point3D.h"
#include "Vector3D.h"
#include "Volume3D.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace kimhyunha;

TEST(MathUtilsTest, DegreesToRadians) {
    EXPECT_NEAR(MathUtils::degrees_to_radians(180.0), M_PI, 1e-9);
    EXPECT_NEAR(MathUtils::degrees_to_radians(90.0), M_PI / 2, 1e-9);
}

TEST(MathUtilsTest, RadiansToDegrees) {
    EXPECT_NEAR(MathUtils::radians_to_degrees(M_PI), 180.0, 1e-9);
    EXPECT_NEAR(MathUtils::radians_to_degrees(0.0), 0.0, 1e-9);
}

TEST(MathUtilsTest, CalculateFps) {
    EXPECT_NEAR(MathUtils::calculate_fps(0.0, 0.1), 10.0, 1e-9);
    EXPECT_EQ(MathUtils::calculate_fps(1.0, 1.0), 0.0);
}

TEST(MathUtilsTest, Clamp) {
    EXPECT_EQ(MathUtils::clamp(5.0, 0.0, 10.0), 5.0);
    EXPECT_EQ(MathUtils::clamp(-3.0, 0.0, 10.0), 0.0);
    EXPECT_EQ(MathUtils::clamp(15.0, 0.0, 10.0), 10.0);
}

TEST(MathUtilsTest, Lerp) {
    EXPECT_NEAR(MathUtils::lerp(0.0, 10.0, 0.5), 5.0, 1e-9);
    EXPECT_NEAR(MathUtils::lerp(0.0, 10.0, 0.0), 0.0, 1e-9);
    EXPECT_NEAR(MathUtils::lerp(0.0, 10.0, 1.0), 10.0, 1e-9);
}

TEST(Point3DTest, ConstructorAndAccessors) {
    Point3D p(1.0, 2.0, 3.0);
    EXPECT_DOUBLE_EQ(p.x(), 1.0);
    EXPECT_DOUBLE_EQ(p.y(), 2.0);
    EXPECT_DOUBLE_EQ(p.z(), 3.0);
}

TEST(Point3DTest, AsTuple) {
    Point3D p(1.0, 2.0, 3.0);
    double out[3];
    p.as_tuple(out);
    EXPECT_DOUBLE_EQ(out[0], 1.0);
    EXPECT_DOUBLE_EQ(out[1], 2.0);
    EXPECT_DOUBLE_EQ(out[2], 3.0);
}

TEST(Volume3DTest, Volume) {
    Volume3D v(2.0, 3.0, 4.0);
    EXPECT_DOUBLE_EQ(v.volume(), 24.0);
    EXPECT_DOUBLE_EQ(v.width(), 2.0);
    EXPECT_DOUBLE_EQ(v.height(), 3.0);
    EXPECT_DOUBLE_EQ(v.depth(), 4.0);
}

TEST(Vector3DTest, InheritsPoint3D) {
    Vector3D v(1.0, 2.0, 3.0);
    EXPECT_DOUBLE_EQ(v.x(), 1.0);
    EXPECT_DOUBLE_EQ(v.y(), 2.0);
    EXPECT_DOUBLE_EQ(v.z(), 3.0);
}

TEST(Vector3DTest, AddSubtract) {
    Vector3D a(1.0, 2.0, 3.0);
    Vector3D b(4.0, 5.0, 6.0);
    Vector3D sum = a + b;
    EXPECT_DOUBLE_EQ(sum.x(), 5.0);
    EXPECT_DOUBLE_EQ(sum.y(), 7.0);
    EXPECT_DOUBLE_EQ(sum.z(), 9.0);
    Vector3D diff = a - b;
    EXPECT_DOUBLE_EQ(diff.x(), -3.0);
    EXPECT_DOUBLE_EQ(diff.y(), -3.0);
    EXPECT_DOUBLE_EQ(diff.z(), -3.0);
}

TEST(Vector3DTest, ScalarMultiplyDivide) {
    Vector3D a(1.0, 2.0, 3.0);
    Vector3D m = a * 2.0;
    EXPECT_DOUBLE_EQ(m.x(), 2.0);
    EXPECT_DOUBLE_EQ(m.y(), 4.0);
    EXPECT_DOUBLE_EQ(m.z(), 6.0);
    Vector3D m2 = 2.0 * a;
    EXPECT_DOUBLE_EQ(m2.x(), 2.0);
    Vector3D d = a / 2.0;
    EXPECT_DOUBLE_EQ(d.x(), 0.5);
    EXPECT_DOUBLE_EQ(d.z(), 1.5);
}

TEST(Vector3DTest, MagnitudeAndNormalized) {
    Vector3D a(1.0, 2.0, 3.0);
    EXPECT_NEAR(a.magnitude(), std::sqrt(14.0), 1e-9);
    Vector3D n = a.normalized();
    EXPECT_NEAR(n.magnitude(), 1.0, 1e-9);
    Vector3D zero(0.0, 0.0, 0.0);
    EXPECT_DOUBLE_EQ(zero.normalized().magnitude(), 0.0);
}

TEST(Vector3DTest, DotAndCross) {
    Vector3D a(1.0, 2.0, 3.0);
    Vector3D b(4.0, 5.0, 6.0);
    EXPECT_DOUBLE_EQ(a.dot(b), 32.0);
    Vector3D c = a.cross(b);
    EXPECT_DOUBLE_EQ(c.x(), -3.0);
    EXPECT_DOUBLE_EQ(c.y(), 6.0);
    EXPECT_DOUBLE_EQ(c.z(), -3.0);
}

TEST(Vector3DTest, Distance) {
    Vector3D a(1.0, 2.0, 3.0);
    Vector3D b(4.0, 5.0, 6.0);
    EXPECT_NEAR(a.distance_to(b), std::sqrt(27.0), 1e-9);
}