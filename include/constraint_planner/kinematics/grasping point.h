#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;
class grasping_point
{
public:
    grasping_point()
    {
        Vector3d z_offset(0, 0, -0.103);

        base_1st = Eigen::Affine3d::Identity();
        base_2nd = Eigen::Affine3d::Identity();
        base_3rd = Eigen::Affine3d::Identity();
        
        base_1st.translation() = Eigen::Vector3d(0, 0.3, 0.6);
        base_2nd.translation() = Eigen::Vector3d(0, -0.3, 0.6);
        base_3rd.translation() = Eigen::Vector3d(1.6, 0.0, 0.6);
        base_3rd.linear() << -1, 0, 0, 
                            0, -1, 0, 
                            0, 0, 1;
        
        bool rotate_chair = true;
        base_main = base_3rd;
        if (rotate_chair)
        {
            std::cout << "***CLOSED CHAIN MODE" << std::endl;
            planning_group = "panda_closed_chain";
            hand_group = "hand_closed_chain";
            base_serve = base_1st;
            obj_Sgrasp.linear() = Quaterniond(0.497631, 0.483366, 0.516627, 0.501814).toRotationMatrix();
            obj_Sgrasp.translation() = Vector3d(-0.4258375, 0.21016605, 0.0207994) + obj_Sgrasp.linear() * z_offset;

            obj_Mgrasp.linear() = Quaterniond(0.507219, 0.492676, -0.507217, -0.492677).toRotationMatrix();
            obj_Mgrasp.translation() = Vector3d(-0.128369 ,0.20230575, 0.017412) + obj_Mgrasp.linear() * z_offset;
        
            Sgrp_obj = obj_Sgrasp.inverse();
            Mgrp_obj = obj_Mgrasp.inverse();
            
            start << 0.8538476617343256, 0.7765987891970887, -1.38718092553011, -1.9162352330353676, 2.693557656819878, 2.209230516957901, -2.8518449420397336,  2.4288973744080127, 0.2356190832102002, -2.6487764272706724, -2.409884568379378, 2.7754012268293335, 2.451555244441547, 2.786489214331766;
            goal << 1.7498062050410104, -0.9361363984733272, -1.718452805762332, -1.5358818576944626, -0.4985517231678765, 3.290268980216566, 0.3288819249851005, 1.853119906824091, -1.7627268325040601, -0.9496175784938026, -2.3353608217025394, -2.5450073491173475, 1.9786896240728153, -1.391123032654605;
        }

        else
        {
            std::cout << "***CHAIR UP MODE" << std::endl;
            planning_group = "panda_chair_up";
            hand_group = "hand_chair_up";
            base_serve = base_2nd;
            obj_Sgrasp.linear() = Quaterniond(-0.0540426, -0.0582587, 0.67793, 0.730818).toRotationMatrix();
            obj_Sgrasp.translation() = Vector3d(-0.6601151, -0.02137197 ,0.02044866) + obj_Sgrasp.linear() * z_offset;

            obj_Mgrasp.linear() = Quaterniond(0.0102825, 0.707032, -0.707032, 0.0102825).toRotationMatrix();
            obj_Mgrasp.translation() = Vector3d(-0.133369, 0.26831887 , 0.01643235) + obj_Mgrasp.linear() * z_offset;
        
            Sgrp_obj = obj_Sgrasp.inverse();
            Mgrp_obj = obj_Mgrasp.inverse();
            
            start << 1.7635811732933235, -1.4411345207422865, -1.964651184080014, -1.7905553615439762, 0.20378384311742412, 1.7390337027885823, -2.800300667744541, -2.507227794231461, -0.23624109784362163, 2.5633123073239905, -2.268388140289912, 0.24936065684482742, 2.4538909693928335, -0.9104041928398361;
            goal <<  -1.3467140413650676, 1.759152969838163, 1.7421234462009596, -2.29020266856861, -0.16968203574810806, 2.3272818650833904, -2.090762525897875, 0.4358071209224652, 0.20526492757492862, -0.8031983820172117, -1.6474676957518197, 0.17823531304271836, 1.7776013922868934, -1.1806954807587389;
        }
        
    }
    
    
    Affine3d base_1st, base_2nd, base_3rd,  base_main, base_serve;
    Affine3d obj_Sgrasp, obj_Mgrasp, Sgrp_obj, Mgrp_obj;
    Matrix<double, 14, 1> start, goal;
    std::string planning_group, hand_group;
};