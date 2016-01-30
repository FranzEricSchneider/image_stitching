#ifndef VECTOR2D_H_INCLUDED
#define VECTOR2D_H_INCLUDED

// Adapted from here: http://www.terathon.com/code/vector2d.html
#include <math.h>
#include <ostream>

class Vector2D
{
    public:
        double x;
        double y;

        Vector2D() {}

        Vector2D(double r, double s)
        {
            x = r;
            y = s;
        }

        Vector2D& Set(double r, double s)
        {
            x = r;
            y = s;
            return (*this);
        }

        double& operator [](long k)
        {
            return ((&x)[k]);
        }

        const double& operator [](long k) const
        {
            return ((&x)[k]);
        }

        Vector2D& operator +=(const Vector2D& v)
        {
            x += v.x;
            y += v.y;
            return (*this);
        }

        Vector2D& operator -=(const Vector2D& v)
        {
            x -= v.x;
            y -= v.y;
            return (*this);
        }

        Vector2D& operator *=(double t)
        {
            x *= t;
            y *= t;
            return (*this);
        }

        Vector2D& operator /=(double t)
        {
            double f = 1.0F / t;
            x *= f;
            y *= f;
            return (*this);
        }

        Vector2D& operator &=(const Vector2D& v)
        {
            x *= v.x;
            y *= v.y;
            return (*this);
        }

        Vector2D operator -(void) const
        {
            return (Vector2D(-x, -y));
        }

        Vector2D operator +(const Vector2D& v) const
        {
            return (Vector2D(x + v.x, y + v.y));
        }

        Vector2D operator -(const Vector2D& v) const
        {
            return (Vector2D(x - v.x, y - v.y));
        }

        Vector2D operator *(double t) const
        {
            return (Vector2D(x * t, y * t));
        }

        Vector2D operator /(double t) const
        {
            double f = 1.0F / t;
            return (Vector2D(x * f, y * f));
        }

        double operator *(const Vector2D& v) const
        {
            return (x * v.x + y * v.y);
        }

        Vector2D operator &(const Vector2D& v) const
        {
            return (Vector2D(x * v.x, y * v.y));
        }

        bool operator ==(const Vector2D& v) const
        {
            return ((x == v.x) && (y == v.y));
        }

        bool operator !=(const Vector2D& v) const
        {
            return ((x != v.x) || (y != v.y));
        }

        Vector2D& Normalize(void)
        {
            return (*this /= sqrtf(x * x + y * y));
        }

        Vector2D& Rotate(double angle);
};


class Point2D : public Vector2D
{
    public:

        Point2D() {}

        Point2D(double r, double s) : Vector2D(r, s) {}

        friend std::ostream& operator<< (std::ostream &out, Point2D &cPoint)
        {
            out << "(" << cPoint.x << ", " <<
                cPoint.y << ")";
            return out;
        }

        Point2D& operator =(const Vector2D& v)
        {
            x = v.x;
            y = v.y;
            return (*this);
        }

        Point2D& operator *=(double t)
        {
            x *= t;
            y *= t;
            return (*this);
        }

        Point2D& operator /=(double t)
        {
            double f = 1.0F / t;
            x *= f;
            y *= f;
            return (*this);
        }

        Point2D operator -(void) const
        {
            return (Point2D(-x, -y));
        }

        Point2D operator +(const Vector2D& v) const
        {
            return (Point2D(x + v.x, y + v.y));
        }

        Point2D operator -(const Vector2D& v) const
        {
            return (Point2D(x - v.x, y - v.y));
        }

        Vector2D operator -(const Point2D& p) const
        {
            return (Vector2D(x - p.x, y - p.y));
        }

        Point2D operator *(double t) const
        {
            return (Point2D(x * t, y * t));
        }

        Point2D operator /(double t) const
        {
            double f = 1.0F / t;
            return (Point2D(x * f, y * f));
        }
};


inline Vector2D operator *(double t, const Vector2D& v)
{
    return (Vector2D(t * v.x, t * v.y));
}

inline Point2D operator *(double t, const Point2D& p)
{
    return (Point2D(t * p.x, t * p.y));
}

inline double Dot(const Vector2D& v1, const Vector2D& v2)
{
    return (v1 * v2);
}

inline double Magnitude(const Vector2D& v)
{
    return (sqrtf(v.x * v.x + v.y * v.y));
}

inline double InverseMag(const Vector2D& v)
{
    return (1.0F / sqrtf(v.x * v.x + v.y * v.y));
}

inline double SquaredMag(const Vector2D& v)
{
    return (v.x * v.x + v.y * v.y);
}

#endif // VECTOR2D_H_INCLUDED
