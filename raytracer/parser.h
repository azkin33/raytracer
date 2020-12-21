#ifndef __HW1__PARSER__
#define __HW1__PARSER__

#include <string>
#include <vector>



namespace parser
{
    //Notice that all the structures are as simple as possible
    //so that you are not enforced to adopt any style or design.
    
    struct Vec2f
    {
        float x, y;
    };

    struct Vec3f
    {
        float x, y, z;
    };

    struct Vec3i
    {
        int x, y, z;
    };

    struct Vec4f
    {
        float x, y, z, w;
    };

    struct Camera
    {
        Vec3f position;
        Vec3f gaze;
        Vec3f up;
        Vec4f near_plane;
        float near_distance;
        int image_width, image_height;
        std::string image_name;
    };

    struct PointLight
    {
        Vec3f position;
        Vec3f intensity;
    };

    struct Material
    {
        Vec3f ambient;
        Vec3f diffuse;
        Vec3f specular;
        Vec3f mirror;
        float phong_exponent;
    };

    
    struct Face
    {
        Vec3f v0;
        Vec3f v1;
        Vec3f v2;
        Vec3f normal;
        int v0_id,v1_id,v2_id;
        int texture_id = 0;
        float beta,gama;
        Vec3f& operator[](int index){
            if(index==0){
                return v0;
            }
            else if(index==1){
                return v1;
            }
            else if(index==2){
                return v2;
            }
        }
    };

   
    struct Mesh
    {
        int material_id;
        int texture_id;
        std::vector<Face> faces;
        std::string transformations;
    };

    struct Triangle
    {
        int material_id;
        int texture_id=0;
        Face indices;
        std::string transformations;
        Vec3f& operator[](int index){
            if(index==0){
                return indices.v0;
            }
            else if(index==1){
                return indices.v1;
            }
            else if(index==2){
                return indices.v2;
            }
        }
    };

    struct Sphere
    {
        int obj_id;
        int material_id;
        int texture_id=0;
        int center_vertex_id;
        Vec3f center;
        float radius;
        std::string transformations;
        Vec3f u,v,w;
    };

    struct Rotation
    {
        float angle, x, y, z;
    };

    struct Texture
    {
        std::string imageName;
        std::string interpolation;
        std::string decalMode;
        std::string appearance;
        unsigned char *arr;
        int width;
        int height;
    };

    struct Scene
    {
        //Data
        Vec3i background_color;
        float shadow_ray_epsilon;
        int max_recursion_depth;
        std::vector<Camera> cameras;
        Vec3f ambient_light;
        std::vector<PointLight> point_lights;
        std::vector<Material> materials;
        std::vector<Vec3f> vertex_data;
        std::vector<Vec2f> tex_coord_data;
        std::vector<Mesh> meshes;
        std::vector<Triangle> triangles;
        std::vector<Sphere> spheres;
        std::vector<Vec3f> translations;
        std::vector<Vec3f> scalings;
        std::vector<Rotation> rotations;
        std::vector<Texture> textures;
        

        //Functions
        void loadFromXml(const std::string& filepath);
    };
}

#endif