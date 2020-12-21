#ifndef __myVector_h__
#define __myVector_h__

#include "parser.h"
#include <cmath>
using namespace parser;
Vec3f add(const Vec3f &a,const Vec3f &b);
Vec3f sub(const Vec3f &a,const Vec3f &b);
Vec3f crossP(const Vec3f &a,const Vec3f &b);
Vec3f normalized(const Vec3f &a);
Vec3f scalarMult(const Vec3f &a,double mul);
Vec3f scalarMult(const Vec3f &a,const Vec3f &b);
double dotP(const Vec3f &a,const Vec3f &b);
double distance(const Vec3f &a,const Vec3f &b);
double getLength(const Vec3f &a);
double det(const Vec3f &a, const Vec3f &b, const Vec3f &c);
struct Ray{
    Vec3f origin;
    Vec3f dir;
};
struct Intersection{
    Vec3f position;
    Vec3f normal;
    Vec3f color;
    bool hasTexture = false;
    int texture_id=0;
    float t;
    int matId;
};

#endif