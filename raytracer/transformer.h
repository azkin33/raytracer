#ifndef __TRANSFORMER_H__
#define __TRANSFORMER_H__
#include "parser.h"
#include "myVector.h"

using namespace parser;


struct Matrix4{
    float elements[16]={0};

    Matrix4& operator+=(const Matrix4 &rhs){
        for(int i=0;i<16;i++){
            this->elements[i] += rhs.elements[i];
        }
    }

    // A*=B -->  B*A
    Matrix4& operator*=(const Matrix4 &rhs){
        float temp[16];
        temp[0]  = rhs.elements[0]*elements[0] + rhs.elements[1]*elements[4] + rhs.elements[2]*elements[8] + rhs.elements[3]*elements[12];
        temp[1]  = rhs.elements[0]*elements[1] + rhs.elements[1]*elements[5] + rhs.elements[2]*elements[9] + rhs.elements[3]*elements[13]; 
        temp[2]  = rhs.elements[0]*elements[2] + rhs.elements[1]*elements[6] + rhs.elements[2]*elements[10] + rhs.elements[3]*elements[14]; 
        temp[3]  = rhs.elements[0]*elements[3] + rhs.elements[1]*elements[7] + rhs.elements[2]*elements[11] + rhs.elements[3]*elements[15]; 

        temp[4]  = rhs.elements[4]*elements[0] + rhs.elements[5]*elements[4] + rhs.elements[6]*elements[8] + rhs.elements[7]*elements[12];
        temp[5]  = rhs.elements[4]*elements[1] + rhs.elements[5]*elements[5] + rhs.elements[6]*elements[9] + rhs.elements[7]*elements[13]; 
        temp[6]  = rhs.elements[4]*elements[2] + rhs.elements[5]*elements[6] + rhs.elements[6]*elements[10] + rhs.elements[7]*elements[14]; 
        temp[7]  = rhs.elements[4]*elements[3] + rhs.elements[5]*elements[7] + rhs.elements[6]*elements[11] + rhs.elements[7]*elements[15]; 

        temp[8]  = rhs.elements[8]*elements[0] + rhs.elements[9]*elements[4] + rhs.elements[10]*elements[8] + rhs.elements[11]*elements[12];
        temp[9]  = rhs.elements[8]*elements[1] + rhs.elements[9]*elements[5] + rhs.elements[10]*elements[9] + rhs.elements[11]*elements[13]; 
        temp[10] = rhs.elements[8]*elements[2] + rhs.elements[9]*elements[6] + rhs.elements[10]*elements[10] + rhs.elements[11]*elements[14]; 
        temp[11] = rhs.elements[8]*elements[3] + rhs.elements[9]*elements[7] + rhs.elements[10]*elements[11] + rhs.elements[11]*elements[15]; 

        temp[12] = rhs.elements[12]*elements[0] + rhs.elements[13]*elements[4] + rhs.elements[14]*elements[8] + rhs.elements[15]*elements[12];
        temp[13] = rhs.elements[12]*elements[1] + rhs.elements[13]*elements[5] + rhs.elements[14]*elements[9] + rhs.elements[15]*elements[13]; 
        temp[14] = rhs.elements[12]*elements[2] + rhs.elements[13]*elements[6] + rhs.elements[14]*elements[10] + rhs.elements[15]*elements[14]; 
        temp[15] = rhs.elements[12]*elements[3] + rhs.elements[13]*elements[7] + rhs.elements[14]*elements[11] + rhs.elements[15]*elements[15]; 

        for(int i=0;i<16;i++){
            elements[i] = temp[i];
        }
        return *this;
    }
    
    
    
};
Matrix4 identity4();
Matrix4 setTranslation(Vec3f translation);
Matrix4 setRotation(Rotation rotation);
Matrix4 setScaling(Vec3f scaling);











#endif