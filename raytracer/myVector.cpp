#include "myVector.h"
#include <cmath>

Vec3f add(const Vec3f &a,const Vec3f &b){
    Vec3f result;
    result.x = a.x + b.x;
    result.y = a.y + b.y;
    result.z = a.z + b.z;
    return result;
}

Vec3f sub(const Vec3f &a,const Vec3f &b){
    Vec3f result;
    result.x = a.x - b.x;
    result.y = a.y - b.y;
    result.z = a.z - b.z;
    return result;
}

Vec3f crossP(const Vec3f &a,const Vec3f &b){
    Vec3f result;
    result.x = a.y*b.z - a.z*b.y;
    result.y = a.z*b.x - a.x*b.z;
    result.z = a.x*b.y - a.y*b.x;
    return result;
}

double getLength(const Vec3f &a){
    return sqrt(a.x*a.x + a.y*a.y + a.z*a.z);
}

Vec3f normalized(const Vec3f &a){
    double length = getLength(a);
    Vec3f result;
    result.x = a.x/length;
    result.y = a.y/length;
    result.z = a.z/length;
    return result;    
}

Vec3f scalarMult(const Vec3f &a,double mul){
    Vec3f result;
    result.x = a.x*mul;
    result.y = a.y*mul;
    result.z = a.z*mul;
    return result;
}

Vec3f scalarMult(const Vec3f &a,const Vec3f &b){
    Vec3f result;
    result.x = a.x*b.x;
    result.y = a.y*b.y;
    result.z = a.z*b.z;
    return result;
}


double dotP(const Vec3f &a,const Vec3f &b){
    double res = a.x*b.x + a.y*b.y + a.z*b.z;
    return res;
}

double distance(const Vec3f &a,const Vec3f &b){
   return sqrt(pow(a.x-b.x,2)+pow(a.y-b.y,2)+pow(a.z-b.z,2)); 
}

double det(const Vec3f &a, const Vec3f &b, const Vec3f &c){
    return a.x*(b.y*c.z-b.z*c.y) - b.x*(a.y*c.z-a.z*c.y) + c.x*(a.y*b.z-a.z*b.y);
}