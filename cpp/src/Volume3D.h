#ifndef KIMHYUNHA_VOLUME3D_H
#define KIMHYUNHA_VOLUME3D_H

namespace kimhyunha {

class Volume3D {
public:
    Volume3D(double width = 0.0, double height = 0.0, double depth = 0.0);
    double width() const;
    double height() const;
    double depth() const;
    double volume() const;

private:
    double width_;
    double height_;
    double depth_;
};

}  // namespace kimhyunha

#endif  // KIMHYUNHA_VOLUME3D_H