#ifndef __GRADIENTMAP_HPP__
#define __GRADIENTMAP_HPP__

#include <map>
#include <math.h>
#ifdef DEBUG
#include <iostream>
#include <numeric>
#include <vector>
#endif

template <typename T>
class bilinear_interpolator {
public:
    T get(T x, T y, std::map<T, std::map<T, T> > m) {

        auto it1 = m.lower_bound(x);
        auto it2 = it1->second.lower_bound(y);

        T x_lower, x_higher, y_lower, y_higher;

        if (it1->first == x) {
            x_lower = x;
            x_higher = x;
        } else {
            x_higher = it1->first;
            it1--;
            x_lower = it1->first;
        }

        if (it2->first == y) {
            y_lower = y;
            y_higher = y;
        } else {
            y_higher = it2->first;
            it2--;
            y_lower = it2->first;
        }

        float xy[4];
        xy[0] = m[x_lower][y_lower];
        xy[1] = m[x_higher][y_lower];
        xy[2] = m[x_lower][y_higher];
        xy[3] = m[x_higher][y_higher];

        return (T)(bilinear(x, y, x_lower, x_higher, y_lower, y_higher, xy));
    }

private:
    float lerp(float x, float x_min, float x_max, float y_min, float y_max) {
        if (x >= x_max) { return y_max; }
        else if (x <= x_min) { return y_min; }
        return y_min + (y_max - y_min)*((x - x_min)/(x_max - x_min));
    }




    float bilinear(T x, T y, T x_lower, T x_higher, T y_lower, T y_higher,
                   float xy[4]) {
        float x1_lerp = lerp(x, x_lower, x_higher, xy[0], xy[2]);
        float x2_lerp = lerp(x, x_lower, x_higher, xy[1], xy[3]);
        float xy_lerp = lerp(y, y_lower, y_higher, x1_lerp, x2_lerp);

#ifdef DEBUG
        std::cout << "wanted: " << x << ", " << y << "\n" <<
            "got_x: " << x_lower << ", " << x_higher << "\n" <<
            "got_y: " << y_lower << ", " << y_higher << "\n";

        std::cout << xy[0] << "\t" << xy[2] << "\n"
                  << xy[1] << "\t" << xy[3] << "\n";

        std::cout << "x1lerp: " << x1_lerp << "\nx2lerp: " << x2_lerp
                  << "\nxylerp: " << xy_lerp << "\n";
#endif

        return xy_lerp;
    }
};

#endif /* __GRADIENTMAP_HPP__ */
