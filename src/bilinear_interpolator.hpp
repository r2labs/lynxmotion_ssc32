#ifndef __GRADIENTMAP_HPP__
#define __GRADIENTMAP_HPP__

#include <map>

template typename T
class bilinear_interpolator {
public:
    int get(T x, T y, std::map<T, std::map<T, T> > m);
private:
    float lerp(float x, float x_min, float x_max, float y_min, float y_max);
};

#endif /* __GRADIENTMAP_HPP__ */







