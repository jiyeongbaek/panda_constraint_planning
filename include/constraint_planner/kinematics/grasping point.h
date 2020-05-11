#include <iostream>
#include <Eigen/Core>

using namespace Eigen;
class grasping_point
{
public:
    grasping_point()
    {
        Vector3d z_offset(0, 0, -0.109);
        obj_Lgrasp.linear() = Quaterniond(0.48089, 0.518406, -0.518406, 0.48089).toRotationMatrix();
        obj_Lgrasp.translation() = Vector3d(-0.417291983962059, 0.385170965965183, 0.189059236695616) + obj_Lgrasp.linear() * z_offset;

        obj_Rgrasp.linear() = Quaterniond(-0.0630359, 0.717459, -0.690838, -0.0634221).toRotationMatrix();
        obj_Rgrasp.translation() = Vector3d(-0.408115020594241, 0.101560465864071, 0.339098439321291) + obj_Rgrasp.linear() * z_offset;

        Lgrasp_obj = obj_Lgrasp.inverse();
    }
    Affine3d obj_Lgrasp, obj_Rgrasp, Lgrasp_obj;
};