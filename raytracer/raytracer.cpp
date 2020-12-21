#include <iostream>
#include "parser.h"
#include "ppm.h"
#include "myVector.h"
#include "transformer.h"
#include <cmath>
#include <thread>
#include <string.h>
#include "jpeg.h"
using namespace parser;
using namespace std;

typedef unsigned char RGB[3];
double EPSILON = 0.0001;
unsigned long int check=0;

Vec3f getSphereTexel(const Intersection &intersection,const Sphere &sphere,const Texture &texture);

Ray computeRay(const Camera &cam,int i,int j){
    Ray ray;
    Vec3f m,q,s,u,gaze,v;
    double l,r,b,t;
    double su,sv;

    l = cam.near_plane.x;
    r = cam.near_plane.y;
    b = cam.near_plane.z;
    t = cam.near_plane.w;


    su = (r-l)*(i+0.5)/cam.image_width;
    sv = (t-b)*(j+0.5)/cam.image_height;
    gaze = normalized(cam.gaze);
    m = add(cam.position,scalarMult(gaze,cam.near_distance));
    
    u = normalized(crossP(gaze,cam.up));
    
    v= crossP(u,gaze);
    q = add(m,add(scalarMult(u,l),scalarMult(v,t)));
    s = add(q,sub(scalarMult(u,su),scalarMult(v,sv)));

    ray.origin = cam.position;
    ray.dir = normalized(sub(s,cam.position));
    return ray;
    
}


double intersectionSphere(const Ray &ray,const Vec3f &center,double radius){
    double a,b,c,delta;
    double t,t1,t2;
    Vec3f o,d;
    o = ray.origin;
    d = ray.dir;
    a = dotP(d,d);
    b = 2 * dotP(d,sub(o,center));
    c = dotP(sub(o,center),sub(o,center)) - radius*radius;
    delta = b*b-4*a*c;
    if(delta<EPSILON) return -1;
    delta = sqrt(delta);
    t1 = (-b+delta)/(2*a);
    t2 = (-b-delta)/(2*a);
    t = min(t1,t2);
    
    
    return t;
}

Vec3f getFaceTexel(const Intersection &intersection,const Face &face,const Texture &texture,const Scene &scene){
    float u,v;
    Vec2f a,b,c;
    a = scene.tex_coord_data[face.v0_id-1];
    b = scene.tex_coord_data[face.v1_id-1];
    c = scene.tex_coord_data[face.v2_id-1];
    float beta,gamma;
    beta = face.beta;
    gamma = face.gama;
    u = a.x + beta*(b.x-a.x) + gamma*(c.x-a.x);
    v = a.y + beta*(b.y-a.y) + gamma*(c.y-a.y);
    int w = texture.width;
    int h = texture.height;
    int i,j;
    if(texture.appearance=="repeat"){
        u = u-floor(u);
        v = v-floor(v);
    }
    Vec3f color;
    u*=w;
    v*=h;
    
    i = int(u);
    j = int(v);
    if(texture.interpolation=="nearest"){
        color.x = texture.arr[w*j*3 +i*3];
        color.y = texture.arr[w*j*3 +i*3+1];
        color.z = texture.arr[w*j*3 +i*3+2];
    }
    else{
        float dx,dy;
        dx = u-floor(u);
        dy = v-floor(v);
        int p,q;
        p = floor(u);
        q = floor(v);
        color.x = texture.arr[w*q*3 + p*3]*(1-dx)*(1-dy) + texture.arr[w*q*3 + (p+1)*3] *dx*(1-dy) + texture.arr[w*(q+1)*3+p*3]*(dy)*(1-dx) + texture.arr[w*(q+1)*3+(p+1)*3]*dx*dy;
        color.y = texture.arr[w*q*3 + p*3+1]*(1-dx)*(1-dy) + texture.arr[w*q*3 + (p+1)*3+1] *dx*(1-dy) + texture.arr[w*(q+1)*3+p*3+1]*(dy)*(1-dx) + texture.arr[w*(q+1)*3+(p+1)*3+1]*dx*dy;
        color.z = texture.arr[w*q*3 + p*3+2]*(1-dx)*(1-dy) + texture.arr[w*q*3 + (p+1)*3+2] *dx*(1-dy) + texture.arr[w*(q+1)*3+p*3+2]*(dy)*(1-dx) + texture.arr[w*(q+1)*3+(p+1)*3+2]*dx*dy;
    }
    
    
    
    return color;
}

double intersectionFace(Face &face,const Ray &ray){
    
    check++;

    Vec3f a,b,c,o,d,aso,asb,asc;
    a = face.v0;
    b = face.v1;
    c = face.v2;
    o = ray.origin;
    d = ray.dir;
    aso = sub(a,o);
    asb = sub(a,b);
    asc = sub(a,c);
    
    double beta,gama,t,detA;
    detA = det(asb,asc,d);
    if(detA==0) return -1;
    t = det(asb,asc,aso)/detA;
    if(t<0) return -1;
    
    gama = det(asb,aso,d)/detA;
    if(gama<-EPSILON || gama>1+EPSILON) return -1;
    beta = det(aso,asc,d)/detA;
    if(beta<-EPSILON|| beta>(1-gama)) return -1;
    face.gama = gama;
    face.beta = beta;
    return t;
}


Intersection intersectionMesh(const Mesh &mesh,const Ray &ray,const Scene &scene){
    Face face;
    int faceCount;
    double t,closest;
    Intersection intersection;
    closest = INFINITY;
    faceCount = mesh.faces.size();
    intersection.t = -1;
    
  

    for(int i=0;i<faceCount;i++){
        face = mesh.faces[i];
        

        t = intersectionFace(face,ray);
        if(t>EPSILON){
            if(t<closest){
                intersection.t = t;
                intersection.matId = mesh.material_id;
                intersection.position = add(ray.origin,scalarMult(ray.dir,t));
                intersection.normal = face.normal;
                closest = t;
                if(mesh.texture_id!=0){
                    intersection.hasTexture = true;
                    intersection.texture_id = mesh.texture_id;
                    intersection.color = getFaceTexel(intersection,face,scene.textures[mesh.texture_id-1],scene);
                }
            }
        }
    }
    
    return intersection;
}

Intersection getClosestIntersection(const vector<Intersection> &intersections){
    Intersection intersection;
    intersection.t = -1;
    if(intersections.size()){
        intersection = intersections[0];
        for(int i=1;i<intersections.size();i++){
            if(intersections[i].t<intersection.t){
                intersection = intersections[i];
            }
        }
    }
    return intersection;
}



Intersection getIntersection(const Scene &scene,const Ray &ray){
    Intersection current;
    vector<Intersection> intersections;
    int triangles,spheres,meshes;
    triangles = scene.triangles.size();
    spheres = scene.spheres.size();
    meshes = scene.meshes.size();
    current.t = -1;
    double t;
    
    for(int i=0;i<triangles;i++){

        Triangle tri = scene.triangles[i];
        

        t = intersectionFace(tri.indices,ray);
        if(t>=0){
            current.t = t;
            current.matId = tri.material_id;
            current.position = add(ray.origin,scalarMult(ray.dir,t));
            current.normal = tri.indices.normal;
            if(tri.texture_id!=0){
                current.hasTexture = true;
                current.texture_id = tri.texture_id;
                current.color = getFaceTexel(current,tri.indices,scene.textures[tri.texture_id-1],scene);
            }
            intersections.push_back(current);
        }
    }

    for(int i=0;i<spheres;i++){
        Sphere sphere = scene.spheres[i];
        Vec3f center = sphere.center;
        double radius = sphere.radius;
        
        t = intersectionSphere(ray,center,radius);
        if(t>=0){
            
            current.t = t;
            current.matId = sphere.material_id;
            current.position = add(ray.origin,scalarMult(ray.dir,t));
            
            current.normal = normalized(sub(current.position,center));
            
            
            if(sphere.texture_id!=0){
                current.hasTexture = true;
                current.texture_id = sphere.texture_id;
                
                current.color = getSphereTexel(current,sphere,scene.textures[current.texture_id-1]);
            }
            intersections.push_back(current);
        }

    }

    for(int i=0;i<meshes;i++){
        Mesh mesh = scene.meshes[i];
        current = intersectionMesh(mesh,ray,scene);
        
        if(current.t>=0) intersections.push_back(current);    
    }
    
    
    return getClosestIntersection(intersections);
}



Vec3f diffuseShading(const Scene &scene,const PointLight &light,const Intersection &intersection){
    Vec3f wi,diffuse;
    Material mat = scene.materials[intersection.matId-1];;
    double dist,dot;
    dist = distance(light.position,intersection.position);
    wi = normalized(sub(light.position,intersection.position));
    dot = dotP(wi,intersection.normal);
    Vec3f c;
    if(dot<0) dot = 0;
    if(intersection.hasTexture==true){
        Texture texture = scene.textures[intersection.texture_id-1];
        if(texture.decalMode=="replace_kd"){
            c.x = intersection.color.x/255;
            c.y = intersection.color.y/255;
            c.z = intersection.color.z/255;
        
        }
        else if(texture.decalMode=="blend_kd"){
            c.x = (mat.diffuse.x + intersection.color.x/255)/2;
            c.y = (mat.diffuse.y + intersection.color.y/255)/2;
            c.z = (mat.diffuse.z + intersection.color.z/255)/2;

        }
        else{

            return intersection.color;
        }
    }
    else{
        c = mat.diffuse;
    }
    diffuse = scalarMult(scalarMult(light.intensity,c),dot/(dist*dist));
    return diffuse;
}


Vec3f specularShading(const Scene &scene,const Ray &ray,const PointLight &light,const Intersection &intersection){
    Vec3f h,specular,wi,wo;
    Material mat = scene.materials[intersection.matId-1];;
    float cosa,dist;
    dist = distance(light.position,intersection.position); 
    wi = normalized(sub(light.position,intersection.position));
    h = normalized(sub(wi,ray.dir));
    cosa = dotP(normalized(intersection.normal),h);
    if(cosa<0) cosa = 0;
    cosa = pow(cosa,mat.phong_exponent);
    specular = scalarMult(mat.specular,scalarMult(light.intensity,cosa/(dist*dist)));
    return specular;
    
}
void transform(Vec3f &p,Matrix4 m){
    Vec4f h;
    h.x = p.x;
    h.y = p.y;
    h.z = p.z;
    h.w = 1;
    Vec4f temp;
    temp.x = m.elements[0]*h.x + m.elements[1]*h.y + m.elements[2]*h.z + m.elements[3]*h.w;
    temp.y = m.elements[4]*h.x + m.elements[5]*h.y + m.elements[6]*h.z + m.elements[7]*h.w;
    temp.z = m.elements[8]*h.x + m.elements[9]*h.y + m.elements[10]*h.z + m.elements[11]*h.w;
    temp.w = m.elements[12]*h.x + m.elements[13]*h.y + m.elements[14]*h.z + m.elements[15]*h.w;

    p.x = temp.x;
    p.y = temp.y;
    p.z = temp.z;
}
Vec3f getSphereTexel(const Intersection &intersection,const Sphere &sphere,const Texture &texture){
    double mypi = 3.141593;
    
    double theta,phi,r;
    double x,y,z;
    Vec3f p;
    r = sphere.radius;
    p.x = intersection.position.x-sphere.center.x;
    p.y = intersection.position.y-sphere.center.y;
    p.z = intersection.position.z-sphere.center.z;
    Matrix4 r1 = identity4();
    
    r1.elements[0] = sphere.u.x;
    r1.elements[1] = sphere.u.y;
    r1.elements[2] = sphere.u.z;
    r1.elements[4] = sphere.v.x;
    r1.elements[5] = sphere.v.y;
    r1.elements[6] = sphere.v.z;
    r1.elements[8] = sphere.w.x;
    r1.elements[9] = sphere.w.y;
    r1.elements[10] = sphere.w.z;
    
    transform(p,r1);
    x = p.x;
    y = p.y;
    z =p.z;
    
    theta = acos(y/r);
    phi = atan2(z,x);
    double u,v;
    
    u = (-phi+mypi) / (2*mypi);
    v = theta /mypi;
    
    u*= texture.width;
    v*= texture.height;
    int w = texture.width;
    int h = texture.height;
    Vec3f color;
    color.x =0;
    color.y =0;
    color.z =0;
    int i = floor(u);
    int j = floor(v);
    if(texture.interpolation=="nearest"){
        color.x = texture.arr[w*j*3 +i*3];
        color.y = texture.arr[w*j*3 +i*3+1];
        color.z = texture.arr[w*j*3 +i*3+2];
    }
    else{
        float dx,dy;
        dx = u-floor(u);
        dy = v-floor(v);
        int p,q;
        p = floor(u);
        q = floor(v);
        color.x = texture.arr[w*q*3 + p*3]*(1-dx)*(1-dy) + texture.arr[w*q*3 + (p+1)*3] *dx*(1-dy) + texture.arr[w*(q+1)*3+p*3]*(dy)*(1-dx) + texture.arr[w*(q+1)*3+(p+1)*3]*dx*dy;
        color.y = texture.arr[w*q*3 + p*3+1]*(1-dx)*(1-dy) + texture.arr[w*q*3 + (p+1)*3+1] *dx*(1-dy) + texture.arr[w*(q+1)*3+p*3+1]*(dy)*(1-dx) + texture.arr[w*(q+1)*3+(p+1)*3+1]*dx*dy;
        color.z = texture.arr[w*q*3 + p*3+2]*(1-dx)*(1-dy) + texture.arr[w*q*3 + (p+1)*3+2] *dx*(1-dy) + texture.arr[w*(q+1)*3+p*3+2]*(dy)*(1-dx) + texture.arr[w*(q+1)*3+(p+1)*3+2]*dx*dy;

    }
    
    
    return color;
}



Vec3f getColor(const Scene &scene,const Intersection &intersection,int depth,const Ray &ray){
    Vec3f color,diffuse,specular;
    Material mat;
    Ray sray;
    PointLight currentLight;
    int lights,triangles,spheres,meshes;
    double e = scene.shadow_ray_epsilon;
    double lightDistance;

    lights = scene.point_lights.size();
    
    if(intersection.t>0){
        mat = scene.materials[intersection.matId-1];
        color = scalarMult(mat.ambient,scene.ambient_light);
        
        
        
        
        Intersection shadow;
        
        for(int i=0;i<lights;i++){
            currentLight = scene.point_lights[i];
            sray.origin = add(intersection.position,scalarMult(intersection.normal,EPSILON));
            sray.dir = sub(currentLight.position,sray.origin);
            shadow = getIntersection(scene,sray);
            if(shadow.t<0||shadow.t>1){
                diffuse = diffuseShading(scene,currentLight,intersection);
                specular = specularShading(scene,ray,currentLight,intersection);
                color = add(color,specular);
                color = add(color,diffuse);
            }
        }
        if(mat.mirror.x>0 || mat.mirror.y>0 || mat.mirror.z>0){
            if(depth>0){
                Vec3f mirrored,wr;
                Ray reflectionRay;
                

                double dot = -2*dotP(ray.dir,intersection.normal);
                wr = normalized(add(scalarMult(intersection.normal,dot),ray.dir));
            

                reflectionRay.origin = add(intersection.position,scalarMult(wr,EPSILON));
                reflectionRay.dir = wr;
                Intersection ref = getIntersection(scene,reflectionRay);
                if(ref.t>EPSILON){
                    mirrored = getColor(scene,ref,depth-1,reflectionRay);
                    color = add(color,scalarMult(mirrored,mat.mirror));
                }
            }
        }
    }
    else{
        color.x = scene.background_color.x;
        color.y = scene.background_color.y;
        color.z = scene.background_color.z;
    }
    return color;
}

void render(vector<Vec3f>& colors,Scene scene,Camera cam,int width,int minHeight,int maxHeight){
    Vec3f color;
    Ray ray;
    Intersection intersection;
    int pixel =minHeight * width;
    for(int y=minHeight;y<maxHeight;y++){
        for(int x=0;x<width;x++){
            
            ray = computeRay(cam,x,y);
            intersection = getIntersection(scene,ray);
            color = getColor(scene,intersection,scene.max_recursion_depth,ray);
            
            if(color.x>255) color.x=255;
            if(color.y>255) color.y=255;
            if(color.z>255) color.z=255;
            
            colors[y*width+x] = color;
        }
    }
}

vector<std::string> getTransformations(std::string transformations){
    vector<std::string> result;
    int k =0;
    for(int i=0;i<transformations.size();i++){
        if(transformations[i]==' '){
            result.push_back(transformations.substr(k,i-k));
            k=i+1;
        }
        else if(i==transformations.size()-1){
            result.push_back(transformations.substr(k,i-k+1)); 
        }
    }
    return result;
}


void transformThemAll(Scene &scene){
    Matrix4 t,s,r,M;
    Vec4f p;
    Vec3f vertice;
    std::string transformations;
    vector<std::string> vec;
    int id;
    char type;
    for(int i=0;i<scene.triangles.size();i++){
        transformations = scene.triangles[i].transformations;
        vec = getTransformations(transformations);
        M = identity4();
        for(int j=0;j<vec.size();j++){
            type = vec[j][0];
            id = stoi(vec[j].substr(1,vec[j].size()-1));

            if(type=='t'){
                t = setTranslation(scene.translations[id-1]);
                M *= t;
            }
            else if(type=='s'){
                s = setScaling(scene.scalings[id-1]);
                M *= s;
            }
            else if(type=='r'){
                r = setRotation(scene.rotations[id-1]);
                M *= r;
            }
        }
        
        
        Triangle triangle = scene.triangles[i];
        for(int j=0;j<3;j++){
            vertice = triangle[j];
            transform(vertice,M);
            triangle[j] = vertice;
        }
        triangle.indices.normal = normalized(crossP(sub(triangle[1],triangle[0]),sub(triangle[2],triangle[0])));
        scene.triangles[i] = triangle;
    }

    for(int i=0;i<scene.spheres.size();i++){
        Sphere sphere = scene.spheres[i];
        vec = getTransformations(sphere.transformations);
        M = identity4();
        Matrix4 R = identity4();
        for(int j=0;j<vec.size();j++){
            type = vec[j][0];
            id = stoi(vec[j].substr(1,vec[j].size()-1));

            if(type=='t'){
                t = setTranslation(scene.translations[id-1]);
                M *= t;
            }
            else if(type=='s'){
                s = setScaling(scene.scalings[id-1]);
                M *= s;
                sphere.radius *= scene.scalings[id-1].x;
            }
            else if(type=='r'){
                r = setRotation(scene.rotations[id-1]);
                M *= r;
                R *= r;
            }
        }
        vertice = sphere.center;
        Vec3f u,v,w;
        u = sphere.u;
        v = sphere.v;
        w = sphere.w;
        transform(vertice,M);
        transform(u,R);
        transform(v,R);
        transform(w,R);
        sphere.v =normalized(v);
        sphere.w = normalized(w);
        sphere.u = normalized(u);
        
        sphere.center = vertice;
        scene.spheres[i] = sphere;
    }

    for(int i=0;i<scene.meshes.size();i++){
        Mesh mesh = scene.meshes[i];
        vec = getTransformations(mesh.transformations);
        M = identity4();
        for(int j=0;j<vec.size();j++){
            type = vec[j][0];
            id = stoi(vec[j].substr(1,vec[j].size()-1));

            if(type=='t'){
                t = setTranslation(scene.translations[id-1]);
                M *= t;
            }
            else if(type=='s'){
                s = setScaling(scene.scalings[id-1]);
                M *= s;
            }
            else if(type=='r'){
                r = setRotation(scene.rotations[id-1]);
                M *= r;
            }
        }
        
        for(int j=0;j<mesh.faces.size();j++){
            Face face = mesh.faces[j];
            for(int k=0;k<3;k++){
                vertice = face[k];
    
                transform(vertice,M);
                face[k] = vertice;

            }
            face.normal = normalized(crossP(sub(face.v1,face.v0),sub(face.v2,face.v0)));
            scene.meshes[i].faces[j] = face;
            
        }
    }
}


int main(int argc,char* argv[])
{
    Scene scene;
    Camera currentCam;
    Ray ray;
    Intersection intersection;
    Vec3f color;
    
    
    scene.loadFromXml(argv[1]);

    EPSILON = scene.shadow_ray_epsilon;
    
    
    
    int cameras,width,height; 
    cameras = scene.cameras.size();
    
    transformThemAll(scene);
    
    
    
    
    /*
    for(int i=0;i<mysize;i++){
        if(i%3==0){
            cout<<endl;
        }
        cout<<int(texture.arr[i])<<",";
    }
    */
    /*auto t1 = std::chrono::high_resolution_clock::now();*/
    

    for(int i=0;i<cameras;i++){
        currentCam = scene.cameras[i];
        width = currentCam.image_width;
        height = currentCam.image_height;
        
        vector<Vec3f> colors(width*height);
        unsigned char* image = new unsigned char [width * height * 3];
        long int pixel = 0;
        
        
        
        int threadCount = std::thread::hardware_concurrency();
        
        //cout<<threadCount<<endl;
        int hDifference = height / threadCount;

        if(threadCount!=0){
            int minWidth,maxWidth;
            int min,max;
            std::thread* threads = new std::thread[threadCount];
            for(int j=0;j<threadCount;j++){
                min = j*hDifference;
                max = (j+1)*hDifference;
                threads[j] = std::thread(&render,std::ref(colors),scene,currentCam,width,min,max);
            }
            for(int j=0;j<threadCount;j++){
                threads[j].join();
            }
            delete[] threads;
        }
        else{
            render(colors,scene,currentCam,width,0,height);
        }
        for(int i=0;i<width;i++){
            for(int j=0;j<height;j++){
                color = colors[i*height+j];
                image[pixel++] = round(color.x);
                image[pixel++] = round(color.y);
                image[pixel++] = round(color.z);
            }
        }
        write_ppm(currentCam.image_name.c_str(),image,width,height);
        delete image;
    }
    
    /*auto t2 = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
    cout<<duration/1000000<<endl;*/
    
    return 0;
}
