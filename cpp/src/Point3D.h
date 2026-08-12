#ifndef KIMHYUNHA_POINT3D_H
#define KIMHYUNHA_POINT3D_H

namespace kimhyunha {

class Point3D {
public:
    Point3D(double x = 0.0, double y = 0.0, double z = 0.0);
    double x() const;
    double y() const;
    double z() const;

    double as_tuple(double out[3]) const;

private:
    double x_;
    double y_;
    double z_;
};

}  // namespace kimhyunha

#endif  // KIMHYUNHA_POINT3D_H