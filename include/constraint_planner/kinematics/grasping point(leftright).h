#include <iostream>
#include <Eigen/Core>

using namespace Eigen;
class grasping_point
{
public:
    grasping_point()
    {
        Vector3d z_offset(0, 0, -0.109);
        obj_Sgrasp.linear() = Quaterniond(0.48089, 0.518406, -0.518406, 0.48089).toRotationMatrix();
        obj_Sgrasp.translation() = Vector3d(-0.417291983962059, 0.385170965965183, 0.189059236695616) + obj_Sgrasp.linear() * z_offset;

        obj_Mgrasp.linear() = Quaterniond(-0.0630359, 0.717459, -0.690838, -0.0634221).toRotationMatrix();
        obj_Mgrasp.translation() = Vector3d(-0.408115020594241, 0.101560465864071, 0.339098439321291) + obj_Mgrasp.linear() * z_offset;

        Lgrasp_obj = obj_Sgrasp.inverse();

        base_serve.translation() = Vector3d(0, 0.3, 0.6);
        base_main.translation() = Vector3d(0, -0.3, 0.6);
        base_serve.linear().setIdentity();
        base_main.linear().setIdentity();
    }
    
    Affine3d obj_Sgrasp, obj_Mgrasp, Lgrasp_obj, base_serve, base_main;
};