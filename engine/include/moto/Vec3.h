#pragma once
#include <cmath>
#include <ostream>

namespace moto {

struct Vec3 {
    double x{}, y{}, z{};

    Vec3() = default;
    Vec3(double x, double y, double z) : x(x), y(y), z(z) {}

    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(double s)       const { return {x*s,   y*s,   z*s  }; }
    Vec3 operator/(double s)       const { return {x/s,   y/s,   z/s  }; }
    Vec3& operator+=(const Vec3& o){ x+=o.x; y+=o.y; z+=o.z; return *this; }
    Vec3& operator*=(double s)     { x*=s;   y*=s;   z*=s;   return *this; }

    double dot(const Vec3& o)  const { return x*o.x + y*o.y + z*o.z; }
    double norm()              const { return std::sqrt(dot(*this)); }
    Vec3   normalized()        const { double n=norm(); return n>1e-12 ? *this/n : Vec3{}; }

    Vec3 cross(const Vec3& o) const {
        return { y*o.z - z*o.y,
                 z*o.x - x*o.z,
                 x*o.y - y*o.x };
    }

    friend Vec3 operator*(double s, const Vec3& v) { return v*s; }
    friend std::ostream& operator<<(std::ostream& os, const Vec3& v) {
        return os << '[' << v.x << ", " << v.y << ", " << v.z << ']';
    }
};

} // namespace moto
