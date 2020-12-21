#include "transformer.h"



Matrix4 setTranslation(Vec3f translation){
    Matrix4 sol;
    sol.elements[0] = 1;
    sol.elements[5] = 1;
    sol.elements[10] = 1;
    sol.elements[15] = 1;
    
    sol.elements[3] = translation.x;
    sol.elements[7] = translation.y;
    sol.elements[11] = translation.z;
    return sol;
}
Matrix4 setScaling(Vec3f scaling){
    Matrix4 sol;
    sol.elements[0] = scaling.x;
    sol.elements[5] = scaling.y;
    sol.elements[10] = scaling.z;
    sol.elements[15] = 1;
    return sol;
}

Matrix4 setRotation(Rotation rotation){
    Matrix4 sol;
    Vec3f u,v,w;
    
    u.x = rotation.x;
    u.y = rotation.y;
    u.z = rotation.z;
    u = normalized(u);
    if(abs(u.x)>abs(u.y)){
        if(abs(u.y)>abs(u.z)){
            v.x = -u.y;
            v.y = u.x;
            v.z = 0;
        }
        else{
            v.x = -u.z;
            v.y = 0;
            v.z = u.x;
        }
    }
    else{
        if(abs(u.x)>abs(u.z)){
            v.x = -u.y;
            v.y = u.x;
            v.z = 0;
        }
        else{
            v.x = 0;
            v.y = -u.z;
            v.z = u.y;
        }
    }
    w = crossP(u,v);
    v = normalized(v);
    w = normalized(w);

    Matrix4 M,Mi,R;
    M.elements[0] = u.x;
    M.elements[1] = u.y;
    M.elements[2] = u.z;
    M.elements[4] = v.x;
    M.elements[5] = v.y;
    M.elements[6] = v.z;
    M.elements[8] = w.x;
    M.elements[9] = w.y;
    M.elements[10] = w.z;
    M.elements[15] = 1;

    Mi.elements[0] = u.x;
    Mi.elements[1] = v.x;
    Mi.elements[2] = w.x;
    Mi.elements[4] = u.y;
    Mi.elements[5] = v.y;
    Mi.elements[6] = w.y;
    Mi.elements[8] = u.z;
    Mi.elements[9] = v.z;
    Mi.elements[10] = w.z;
    Mi.elements[15] = 1;

    float mypi = 3.14;
    R.elements[0] = 1;
    R.elements[5] = cos(rotation.angle*mypi/180);
    R.elements[6] = -sin(rotation.angle*mypi/180);
    R.elements[9] = sin(rotation.angle*mypi/180);
    R.elements[10] = cos(rotation.angle*mypi/180);
    R.elements[15] = 1;
    
    M*=R;
    M*=Mi;
    return M;
}

Matrix4 identity4(){
    Matrix4 i;
    i.elements[0] = 1;
    i.elements[5] = 1;
    i.elements[10] = 1;
    i.elements[15] = 1;
    return i;
}