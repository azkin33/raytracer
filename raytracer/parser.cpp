#include "parser.h"
#include "myVector.h"
#include "tinyxml2.h"
#include <sstream>
#include <stdexcept>
#include <iostream>
#include "jpeg.h"

void parser::Scene::loadFromXml(const std::string& filepath)
{
    tinyxml2::XMLDocument file;
    std::stringstream stream;
    int objid = 0;
    auto res = file.LoadFile(filepath.c_str());
    if (res)
    {
        throw std::runtime_error("Error: The xml file cannot be loaded.");
    }
    
    auto root = file.FirstChild();
    if (!root)
    {
        throw std::runtime_error("Error: Root is not found.");
    }
    
    //Get BackgroundColor
    auto element = root->FirstChildElement("BackgroundColor");
    if (element)
    {
        stream << element->GetText() << std::endl;
    }
    else
    {
        stream << "0 0 0" << std::endl;
    }
    stream >> background_color.x >> background_color.y >> background_color.z;
    
    //Get ShadowRayEpsilon
    element = root->FirstChildElement("ShadowRayEpsilon");
    if (element)
    {
        stream << element->GetText() << std::endl;
    }
    else
    {
        stream << "0.001" << std::endl;
    }
    stream >> shadow_ray_epsilon;

    //Get MaxRecursionDepth
    element = root->FirstChildElement("MaxRecursionDepth");
    if (element)
    {
        stream << element->GetText() << std::endl;
    }
    else
    {
        stream << "0" << std::endl;
    }
    stream >> max_recursion_depth;
    
    //Get Cameras
    element = root->FirstChildElement("Cameras");
    element = element->FirstChildElement("Camera");
    Camera camera;
    while (element)
    {
        auto child = element->FirstChildElement("Position");
        stream << child->GetText() << std::endl;
        child = element->FirstChildElement("Gaze");
        stream << child->GetText() << std::endl;
        child = element->FirstChildElement("Up");
        stream << child->GetText() << std::endl;
        child = element->FirstChildElement("NearPlane");
        stream << child->GetText() << std::endl;
        child = element->FirstChildElement("NearDistance");
        stream << child->GetText() << std::endl;
        child = element->FirstChildElement("ImageResolution");
        stream << child->GetText() << std::endl;
        child = element->FirstChildElement("ImageName");
        stream << child->GetText() << std::endl;

        stream >> camera.position.x >> camera.position.y >> camera.position.z;
        stream >> camera.gaze.x >> camera.gaze.y >> camera.gaze.z;
        stream >> camera.up.x >> camera.up.y >> camera.up.z;
        stream >> camera.near_plane.x >> camera.near_plane.y >> camera.near_plane.z >> camera.near_plane.w;
        stream >> camera.near_distance;
        stream >> camera.image_width >> camera.image_height;
        stream >> camera.image_name;
        
        cameras.push_back(camera);
        element = element->NextSiblingElement("Camera");
    }

    //Get Lights
    element = root->FirstChildElement("Lights");
    auto child = element->FirstChildElement("AmbientLight");
    stream << child->GetText() << std::endl;
    stream >> ambient_light.x >> ambient_light.y >> ambient_light.z;
    element = element->FirstChildElement("PointLight");
    PointLight point_light;
    while (element)
    {
        child = element->FirstChildElement("Position");
        stream << child->GetText() << std::endl;
        child = element->FirstChildElement("Intensity");
        stream << child->GetText() << std::endl;

        stream >> point_light.position.x >> point_light.position.y >> point_light.position.z;
        stream >> point_light.intensity.x >> point_light.intensity.y >> point_light.intensity.z;

        point_lights.push_back(point_light);
        element = element->NextSiblingElement("PointLight");
    }
    
    //Get Materials
    element = root->FirstChildElement("Materials");
    element = element->FirstChildElement("Material");
    Material material;
    while (element)
    {
        Material material;
        child = element->FirstChildElement("MirrorReflectance");

        if (child) {
            stream << child->GetText() << std::endl;
            stream >> material.mirror.x >> material.mirror.y >> material.mirror.z;
        }
        else{
            material.mirror.x = 0;
            material.mirror.y = 0;
            material.mirror.z = 0;
        }
        child = element->FirstChildElement("AmbientReflectance");
        stream << child->GetText() << std::endl;
        child = element->FirstChildElement("DiffuseReflectance");
        stream << child->GetText() << std::endl;
        child = element->FirstChildElement("SpecularReflectance");
        stream << child->GetText() << std::endl;        
        child = element->FirstChildElement("PhongExponent");
        stream << child->GetText() << std::endl;

        stream >> material.ambient.x >> material.ambient.y >> material.ambient.z;
        stream >> material.diffuse.x >> material.diffuse.y >> material.diffuse.z;
        stream >> material.specular.x >> material.specular.y >> material.specular.z;
        stream >> material.phong_exponent;

        materials.push_back(material);
        element = element->NextSiblingElement("Material");
    }
    

    //Get Textures
    element = root->FirstChildElement("Textures");

    if (element) {
        element = element->FirstChildElement("Texture");
        
        while (element) {
            Texture texture;
            child = element->FirstChildElement("ImageName");
            stream << child->GetText() << std::endl;
            child = element->FirstChildElement("Interpolation");
            stream << child->GetText() << std::endl;
            child = element->FirstChildElement("DecalMode");
            stream << child->GetText() << std::endl;
            child = element->FirstChildElement("Appearance");
            stream << child->GetText() << std::endl;

            stream >> texture.imageName;
            stream >> texture.interpolation;
            stream >> texture.decalMode;
            stream >> texture.appearance;

            
            int w,h;
            
            char imageChar[texture.imageName.size()];
            strcpy(imageChar,texture.imageName.c_str());
            read_jpeg_header(imageChar,w,h);
            texture.arr = new unsigned char[w*h*3];
            read_jpeg(imageChar,texture.arr,w,h);
            
            texture.height = h;
            texture.width = w;
    
            
            textures.push_back(texture);
            element = element->NextSiblingElement("Texture");
        }
    }

    //Get Transformations
    element = root->FirstChildElement("Transformations");

    if (element) {
        child = element->FirstChildElement();

        while (child) {
            const char* tagName = child->Name();

            if (! strcmp(tagName, "Translation")) {
                Vec3f translation;
                stream << child->GetText() << std::endl;
                stream >> translation.x >> translation.y >> translation.z;

                translations.push_back(translation);
            }        

            else if (! strcmp(tagName, "Scaling")) {
                Vec3f scaling;      
                stream << child->GetText() << std::endl;
                stream >> scaling.x >> scaling.y >> scaling.z;

                scalings.push_back(scaling);
            } 

            else if (! strcmp(tagName, "Rotation")) {
                Rotation rotation;
                
                stream << child->GetText() << std::endl;
                stream >> rotation.angle >> rotation.x >> rotation.y >> rotation.z;

                rotations.push_back(rotation);
            } 

            else;

            child = child->NextSiblingElement();
        }
    }

    //Get VertexData
    element = root->FirstChildElement("VertexData");
    stream << element->GetText() << std::endl;
    Vec3f vertex;
    while (!(stream >> vertex.x).eof())
    {
        stream >> vertex.y >> vertex.z;
        vertex_data.push_back(vertex);
    }
    stream.clear();

    //Get TexCoordData
    element = root->FirstChildElement("TexCoordData");
    
    if (element) {
        stream << element->GetText() << std::endl;
        Vec2f tex_coord;
        while (!(stream >> tex_coord.x).eof())
        {
            stream >> tex_coord.y;
            tex_coord_data.push_back(tex_coord);
        }
        
        stream.clear();
    }
    
    //Get Meshes
    element = root->FirstChildElement("Objects");
    element = element->FirstChildElement("Mesh");
    Mesh mesh;
    while (element)
    {
        child = element->FirstChildElement("Material");
        stream << child->GetText() << std::endl;
        stream >> mesh.material_id;

        child = element->FirstChildElement("Texture");

        if (child) {
            stream << child->GetText() << std::endl;
            stream >> mesh.texture_id;
        }
        else{
            mesh.texture_id = 0;
        }

        child = element->FirstChildElement("Transformations");
        
        if (child) {
            const char* temp = child->GetText();
            int len = strlen(temp);
            mesh.transformations.assign(temp, len);
        }

        child = element->FirstChildElement("Faces");
        stream << child->GetText() << std::endl;
        Face face;
        int v0,v1,v2;
        while (!(stream >> v0).eof())
        {
            stream >> v1 >> v2;
            face.v0_id = v0;
            face.v1_id = v1;
            face.v2_id = v2;
            face.v0 = vertex_data[v0-1];
            face.v1 = vertex_data[v1-1];
            face.v2 = vertex_data[v2-1];
            face.normal = normalized(crossP(sub(face.v1,face.v0),sub(face.v2,face.v0)));
            face.texture_id = mesh.texture_id;
            mesh.faces.push_back(face);
        }
        stream.clear();

        meshes.push_back(mesh);
        mesh.faces.clear();
        element = element->NextSiblingElement("Mesh");
    }
    stream.clear();

    //Get Triangles
    element = root->FirstChildElement("Objects");
    element = element->FirstChildElement("Triangle");
    Triangle triangle;
    
    while (element)
    {
        child = element->FirstChildElement("Material");
        stream << child->GetText() << std::endl;
        stream >> triangle.material_id;

        child = element->FirstChildElement("Texture");

        if (child) {
            stream << child->GetText() << std::endl;
            stream >> triangle.texture_id;
        }
        else{
            triangle.texture_id = 0;
        }

        child = element->FirstChildElement("Transformations");
        
        if (child) {
            const char* temp = child->GetText();
            int len = strlen(temp);
            triangle.transformations.assign(temp, len);
        }

        child = element->FirstChildElement("Indices");
        stream << child->GetText() << std::endl;
        int v0,v1,v2;
        stream >> v0 >> v1 >> v2;
        triangle.indices.v0_id = v0;
        triangle.indices.v1_id = v1;
        triangle.indices.v2_id = v2;

        triangle.indices.v0 = vertex_data[v0-1];
        triangle.indices.v1 = vertex_data[v1-1];
        triangle.indices.v2 = vertex_data[v2-1];
        triangle.indices.texture_id = triangle.texture_id;
        triangle.indices.normal = normalized(crossP(sub(triangle.indices.v1,triangle.indices.v0),sub(triangle.indices.v2,triangle.indices.v0)));
        

        triangles.push_back(triangle);
        element = element->NextSiblingElement("Triangle");
    }

    //Get Spheres
    element = root->FirstChildElement("Objects");
    element = element->FirstChildElement("Sphere");
    Sphere sphere;
    while (element)
    {
        child = element->FirstChildElement("Material");
        stream << child->GetText() << std::endl;
        stream >> sphere.material_id;

        child = element->FirstChildElement("Texture");

        if (child) {
            stream << child->GetText() << std::endl;
            stream >> sphere.texture_id;
        }
        else{
            sphere.texture_id = 0;
        }
        child = element->FirstChildElement("Transformations");

        if (child) {
            const char* temp = child->GetText();
            int len = strlen(temp);
            sphere.transformations.assign(temp, len);
        }
        
        child = element->FirstChildElement("Center");
        stream << child->GetText() << std::endl;
        stream >> sphere.center_vertex_id;

        sphere.center = vertex_data[sphere.center_vertex_id-1];

        child = element->FirstChildElement("Radius");
        stream << child->GetText() << std::endl;
        stream >> sphere.radius;


        sphere.obj_id = objid;
        objid++;
        sphere.u.x = 1;
        sphere.u.y = 0;
        sphere.u.z = 0;

        sphere.v.x = 0;
        sphere.v.y = 1;
        sphere.v.z = 0;

        sphere.w.x = 0;
        sphere.w.y = 0;
        sphere.w.z = 1;

        spheres.push_back(sphere);
        element = element->NextSiblingElement("Sphere");
    }
}
