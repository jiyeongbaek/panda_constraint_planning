#pragma once

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/stride/STRIDE.h>
#include <ompl/tools/benchmark/Benchmark.h>

#include <boost/math/constants/constants.hpp>
#include <boost/format.hpp>
#include <fstream>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <chrono>
using namespace Eigen;
// simply use a random projection
class KinematicChainProjector : public ompl::base::ProjectionEvaluator
{
public:
    KinematicChainProjector(const ompl::base::StateSpace *space) : ompl::base::ProjectionEvaluator(space)
    {
        int dimension = std::max(2, (int)ceil(log((double)space->getDimension()))); //ceil : 올림함수, dimension = 3
        projectionMatrix_.computeRandom(space->getDimension(), dimension);
    }
    unsigned int getDimension() const override
    {
        return projectionMatrix_.mat.rows();
    }
    void project(const ompl::base::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        std::vector<double> v(space_->getDimension());
        space_->copyToReals(v, state);
        projectionMatrix_.project(&v[0], projection);
    }

protected:
    /* A projection matrix -- it allows multiplication of
            real vectors by a specified matrix. The matrix can also be
            randomly generated. */
    ompl::base::ProjectionMatrix projectionMatrix_;
};

class KinematicChainSpace : public ompl::base::RealVectorStateSpace
{
public:
    KinematicChainSpace(unsigned int numLinks)
      : ompl::base::RealVectorStateSpace(numLinks)
    {
        ompl::base::RealVectorBounds bounds(numLinks);
        
        for (int i = 0; i < 2; i++)
        {
            bounds.setLow(0 + i * 7, -2.8973);
            bounds.setHigh(0 + i * 7, 2.8973);

            bounds.setLow(1 + i * 7, -1.7628);
            bounds.setHigh(1 + i * 7, 1.7628);

            bounds.setLow(2 + i * 7, -2.8973);
            bounds.setHigh(2 + i * 7, 2.8973);

            bounds.setLow(3 + i * 7, -3.0718);
            bounds.setHigh(3 + i * 7, -0.0698);

            bounds.setLow(4 + i * 7, -2.8973);
            bounds.setHigh(4 + i * 7, 2.8973);

            bounds.setLow(5 + i * 7, -0.0175);
            bounds.setHigh(5 + i * 7, 3.7525);

            bounds.setLow(6 + i * 7, -2.8973);
            bounds.setHigh(6 + i * 7, 2.8973);
        }
        setBounds(bounds);
        // std::cout << "  - min: ";
        // for (unsigned int i = 0; i < dimension_; ++i)
        //     std::cout << bounds.low[i] << " ";
        // std::cout << std::endl;
        // std::cout << "  - max: ";
        // for (unsigned int i = 0; i < dimension_; ++i)
        //     std::cout << bounds.high[i] << "  ";
        // std::cout << std::endl;
        
        type_ = ompl::base::STATE_SPACE_SO2;
        // type_ = ompl::base::STATE_SPACE_REAL_VECTOR;
    }

    void registerProjections() override
    {
        registerDefaultProjection(std::make_shared<KinematicChainProjector>(this));
    }


    // void enforceBounds(ompl::base::State *state) const override
    // {
    //     auto *statet = state->as<StateType>();
    //     for (int i = 0; i < 2; i++)
    //     {
    //         double v0 = fmod(statet->values[0 + i*7], 2.0 * 2.8973);
    //         double v1 = fmod(statet->values[1 + i*7], 2.0 * 1.7628);
    //         double v2 = fmod(statet->values[2 + i*7], 2.0 * 2.8973);
    //         double v3 = fmod(statet->values[3 + i*7], -0.0698 -3.0718);
    //         double v4 = fmod(statet->values[4 + i*7], 2.0 * 2.8973);
    //         double v5 = fmod(statet->values[5 + i*7], 3.7525-0.0175);
    //         double v6 = fmod(statet->values[6 + i*7], 2.0 * 2.8973);

    //         if (v0 < -2.8973)
    //             v0 += 2.0 * 2.8973;
    //         else if (v0 > 2.8973)
    //             v0 -= 2.0 * 2.8973;
    //         statet->values[0 + i*7] = v0;
            
    //         if (v1 < -1.7628)
    //             v1 += 2.0 * 1.7628;
    //         else if (v1 > 1.7628)
    //             v1 -= 2.0 * 1.7628;    
    //         statet->values[1 + i*7] = v1;

    //         if (v2 < -2.8973)
    //             v2 += 2.0 * 2.8973;
    //         else if (v2 > 2.8973)
    //             v2 -= 2.0 * 2.8973;
    //         statet->values[2 + i*7] = v2;

    //         if (v3 < -3.0718)
    //             v3 += -0.0698 -3.0718;
    //         if (v3 > -0.0698)
    //             v3 -= -0.0698 -3.0718;
    //         statet->values[3 + i*7] = v3;

    //         if (v4 < -2.8973)
    //             v4 += 2.0 * 2.8973;
    //         else if (v4 > -2.8973)
    //             v4 -= 2.0 * 2.8973;    
    //         statet->values[4 + i*7] = v4;

    //         if (v5 < -0.0175)
    //             v5 += 3.7525-0.0175;
    //         else if (v5 > 3.7525)
    //             v5 -= 3.7525-0.0175;
    //         statet->values[5 + i*7] = v5;

    //         if (v6 < -2.8973)
    //             v6 += 2.0 * 2.8973;
    //         if (v6 > -2.8973)
    //             v6 -= 2.0 * 2.8973;
    //         statet->values[6 + i*7] = v6;
    //     }
    // }

    // double distance(const ompl::base::State *state1, const ompl::base::State *state2) const override
    // {
    //     const auto *cstate1 = state1->as<StateType>();
    //     const auto *cstate2 = state2->as<StateType>();
    // }
    bool equalStates(const ompl::base::State *state1, const ompl::base::State *state2) const override
    {

        const double *s1 = static_cast<const StateType *>(state1)->values;
        const double *s2 = static_cast<const StateType *>(state2)->values;
        for (unsigned int i = 0; i < dimension_; ++i)
        {
            double diff = (*s1++) - (*s2++);
            if (fabs(diff) > 1e-10) // 10
                return false;
        }
        return true;
    }

    /* 	Computes the state that lies at time t in [0, 1] on the segment that connects from state to to state. 
    The memory location of state is not required to be different from the memory of either from or to. */

    // void interpolate(const ompl::base::State *from, const ompl::base::State *to, const double t,
    //                  ompl::base::State *state) const override
    // {
    //     const auto *fromt = from->as<StateType>();
    //     const auto *tot = to->as<StateType>();
    //     auto *statet = state->as<StateType>();

    //     for (unsigned int i = 0; i < dimension_; ++i)
    //     {
    //         double diff = tot->values[i] - fromt->values[i];
    //         if (fabs(diff) <= boost::math::constants::pi<double>())
    //             statet->values[i] = fromt->values[i] + diff * t;
    //         else
    //         {
    //             if (diff > 0.0)
    //                 diff = 2.0 * boost::math::constants::pi<double>() - diff;
    //             else
    //                 diff = -2.0 * boost::math::constants::pi<double>() - diff;

    //             statet->values[i] = fromt->values[i] - diff * t;
    //             if (statet->values[i] > boost::math::constants::pi<double>())
    //                 statet->values[i] -= 2.0 * boost::math::constants::pi<double>();
    //             else if (statet->values[i] < -boost::math::constants::pi<double>())
    //                 statet->values[i] += 2.0 * boost::math::constants::pi<double>();
    //         }
    //     }
    // }

protected:
   
};

#include <moveit/collision_detection/collision_common.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/utils/robot_model_test_utils.h>

#include <moveit/collision_detection_fcl/collision_common.h>
#include <moveit/collision_detection_fcl/collision_world_fcl.h>
#include <moveit/collision_detection_fcl/collision_robot_fcl.h>

#include <urdf_parser/urdf_parser.h>
#include <geometric_shapes/shape_operations.h>

#include <sstream>
#include <algorithm>
#include <ctype.h>
#include <fstream>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/shape_operations.h>

typedef collision_detection::CollisionWorldFCL DefaultCWorldType;
typedef collision_detection::CollisionRobotFCL DefaultCRobotType;
const std::string panda_joint_names[14] = {"panda_left_joint1", "panda_left_joint2", "panda_left_joint3", "panda_left_joint4", "panda_left_joint5", "panda_left_joint6", "panda_left_joint7",
                            "panda_right_joint1", "panda_right_joint2", "panda_right_joint3", "panda_right_joint4", "panda_right_joint5", "panda_right_joint6", "panda_right_joint7"};
/** \brief Brings the panda robot in user defined home position */
inline void setToHome(robot_state::RobotState& panda_state)
{
  panda_state.setToDefaultValues();
  double joint2 = -0.785;
  double joint4 = -2.356;
  double joint6 = 1.571;
  double joint7 = 0.785;
  panda_state.setJointPositions("panda_left_joint2", &joint2);
  panda_state.setJointPositions("panda_right_joint2", &joint2);
  panda_state.setJointPositions("panda_left_joint4", &joint4);
  panda_state.setJointPositions("panda_right_joint4", &joint4);
  panda_state.setJointPositions("panda_left_joint6", &joint6);
  panda_state.setJointPositions("panda_right_joint6", &joint6);
  panda_state.setJointPositions("panda_left_joint7", &joint7);
  panda_state.setJointPositions("panda_right_joint7", &joint7);
  panda_state.update();
}

class PandaCollisionCheck
{
public:
  PandaCollisionCheck()
  {
    robot_model_ = moveit::core::loadTestingRobotModel("dual_panda");
    robot_model_ok_ = static_cast<bool>(robot_model_);
    acm_.reset(new collision_detection::AllowedCollisionMatrix(robot_model_->getLinkModelNames(), false));

    acm_->setEntry("base", "panda_left_link0", true);
    acm_->setEntry("base", "panda_left_link1", true);
    acm_->setEntry("base", "panda_left_link2", true);
    acm_->setEntry("base", "panda_left_link3", true);
    acm_->setEntry("base", "panda_left_link4", true);

    acm_->setEntry("base", "panda_right_link0", true);
    acm_->setEntry("base", "panda_right_link1", true);
    acm_->setEntry("base", "panda_right_link2", true);
    acm_->setEntry("base", "panda_right_link3", true);
    acm_->setEntry("base", "panda_right_link4", true);

    acm_->setEntry("panda_left_hand" ,"panda_left_leftfinger" , true);
    acm_->setEntry("panda_left_hand" ,"panda_left_link3" , true);
    acm_->setEntry("panda_left_hand" ,"panda_left_link4" , true);
    acm_->setEntry("panda_left_hand" ,"panda_left_link5" , true);
    acm_->setEntry("panda_left_hand" ,"panda_left_link6" , true);
    acm_->setEntry("panda_left_hand" ,"panda_left_link7" , true);
    acm_->setEntry("panda_left_hand" ,"panda_left_rightfinger" , true);
    acm_->setEntry("panda_left_hand" ,"panda_right_link0" , true);
    acm_->setEntry("panda_left_leftfinger" ,"panda_left_link3" , true);
    acm_->setEntry("panda_left_leftfinger" ,"panda_left_link4" , true);
    acm_->setEntry("panda_left_leftfinger" ,"panda_left_link6" , true);
    acm_->setEntry("panda_left_leftfinger" ,"panda_left_link7" , true);
    acm_->setEntry("panda_left_leftfinger" ,"panda_left_rightfinger" , true);
    acm_->setEntry("panda_left_leftfinger" ,"panda_right_link0" , true);
    acm_->setEntry("panda_left_link0" ,"panda_left_link1" , true);
    acm_->setEntry("panda_left_link0" ,"panda_left_link2" , true);
    acm_->setEntry("panda_left_link0" ,"panda_left_link3" , true);
    acm_->setEntry("panda_left_link0" ,"panda_left_link4" , true);
    acm_->setEntry("panda_left_link0" ,"panda_right_hand" , true);
    acm_->setEntry("panda_left_link0" ,"panda_right_leftfinger" , true);
    acm_->setEntry("panda_left_link0" ,"panda_right_link0" , true);
    acm_->setEntry("panda_left_link0" ,"panda_right_link1" , true);
    acm_->setEntry("panda_left_link0" ,"panda_right_link2" , true);
    acm_->setEntry("panda_left_link0" ,"panda_right_link3" , true);
    acm_->setEntry("panda_left_link0" ,"panda_right_link4" , true);
    acm_->setEntry("panda_left_link0" ,"panda_right_link5" , true);
    acm_->setEntry("panda_left_link0" ,"panda_right_link6" , true);
    acm_->setEntry("panda_left_link0" ,"panda_right_link7" , true);
    acm_->setEntry("panda_left_link1" ,"panda_left_link2" , true);
    acm_->setEntry("panda_left_link1" ,"panda_left_link3" , true);
    acm_->setEntry("panda_left_link1" ,"panda_left_link4" , true);
    acm_->setEntry("panda_left_link1" ,"panda_right_link0" , true);
    acm_->setEntry("panda_left_link1" ,"panda_right_link1" , true);
    acm_->setEntry("panda_left_link1" ,"panda_right_link2" , true);
    acm_->setEntry("panda_left_link1" ,"panda_right_link3" , true);
    acm_->setEntry("panda_left_link1" ,"panda_right_link4" , true);
    acm_->setEntry("panda_left_link1" ,"panda_right_link5" , true);
    acm_->setEntry("panda_left_link1" ,"panda_right_link6" , true);
    acm_->setEntry("panda_left_link1" ,"panda_right_link7" , true);
    acm_->setEntry("panda_left_link2" ,"panda_left_link3" , true);
    acm_->setEntry("panda_left_link2" ,"panda_left_link4" , true);
    acm_->setEntry("panda_left_link2" ,"panda_right_link0" , true);
    acm_->setEntry("panda_left_link2" ,"panda_right_link1" , true);
    acm_->setEntry("panda_left_link2" ,"panda_right_link2" , true);
    acm_->setEntry("panda_left_link2" ,"panda_right_link3" , true);
    acm_->setEntry("panda_left_link2" ,"panda_right_link4" , true);
    acm_->setEntry("panda_left_link2" ,"panda_right_link5" , true);
    acm_->setEntry("panda_left_link3" ,"panda_left_link4" , true);
    acm_->setEntry("panda_left_link3" ,"panda_left_link5" , true);
    acm_->setEntry("panda_left_link3" ,"panda_left_link6" , true);
    acm_->setEntry("panda_left_link3" ,"panda_left_link7" , true);
    acm_->setEntry("panda_left_link3" ,"panda_left_rightfinger" , true);
    acm_->setEntry("panda_left_link3" ,"panda_right_link0" , true);
    acm_->setEntry("panda_left_link3" ,"panda_right_link1" , true);
    acm_->setEntry("panda_left_link3" ,"panda_right_link2" , true);
    acm_->setEntry("panda_left_link3" ,"panda_right_link3" , true);
    acm_->setEntry("panda_left_link3" ,"panda_right_link4" , true);
    acm_->setEntry("panda_left_link4" ,"panda_left_link5" , true);
    acm_->setEntry("panda_left_link4" ,"panda_left_link6" , true);
    acm_->setEntry("panda_left_link4" ,"panda_left_link7" , true);
    acm_->setEntry("panda_left_link4" ,"panda_left_rightfinger" , true);
    acm_->setEntry("panda_left_link4" ,"panda_right_link0" , true);
    acm_->setEntry("panda_left_link4" ,"panda_right_link1" , true);
    acm_->setEntry("panda_left_link4" ,"panda_right_link2" , true);
    acm_->setEntry("panda_left_link4" ,"panda_right_link3" , true);
    acm_->setEntry("panda_left_link4" ,"panda_right_link4" , true);
    acm_->setEntry("panda_left_link5" ,"panda_left_link6" , true);
    acm_->setEntry("panda_left_link5" ,"panda_left_link7" , true);
    acm_->setEntry("panda_left_link5" ,"panda_right_link0" , true);
    acm_->setEntry("panda_left_link5" ,"panda_right_link1" , true);
    acm_->setEntry("panda_left_link5" ,"panda_right_link2" , true);
    acm_->setEntry("panda_left_link6" ,"panda_left_link7" , true);
    acm_->setEntry("panda_left_link6" ,"panda_left_rightfinger" , true);
    acm_->setEntry("panda_left_link6" ,"panda_right_link0" , true);
    acm_->setEntry("panda_left_link6" ,"panda_right_link1" , true);
    acm_->setEntry("panda_left_link7" ,"panda_left_rightfinger" , true);
    acm_->setEntry("panda_left_link7" ,"panda_right_link0" , true);
    acm_->setEntry("panda_left_link7" ,"panda_right_link1" , true);
    acm_->setEntry("panda_left_rightfinger" ,"panda_right_link0" , true);
    acm_->setEntry("panda_left_rightfinger" ,"panda_right_rightfinger" , true);
    acm_->setEntry("panda_right_hand" ,"panda_right_leftfinger" , true);
    acm_->setEntry("panda_right_hand" ,"panda_right_link3" , true);
    acm_->setEntry("panda_right_hand" ,"panda_right_link4" , true);
    acm_->setEntry("panda_right_hand" ,"panda_right_link5" , true);
    acm_->setEntry("panda_right_hand" ,"panda_right_link6" , true);
    acm_->setEntry("panda_right_hand" ,"panda_right_link7" , true);
    acm_->setEntry("panda_right_hand" ,"panda_right_rightfinger" , true);
    acm_->setEntry("panda_right_leftfinger" ,"panda_right_link3" , true);
    acm_->setEntry("panda_right_leftfinger" ,"panda_right_link4" , true);
    acm_->setEntry("panda_right_leftfinger" ,"panda_right_link6" , true);
    acm_->setEntry("panda_right_leftfinger" ,"panda_right_link7" , true);
    acm_->setEntry("panda_right_leftfinger" ,"panda_right_rightfinger" , true);
    acm_->setEntry("panda_right_link0" ,"panda_right_link1" , true);
    acm_->setEntry("panda_right_link0" ,"panda_right_link2" , true);
    acm_->setEntry("panda_right_link0" ,"panda_right_link3" , true);
    acm_->setEntry("panda_right_link0" ,"panda_right_link4" , true);
    acm_->setEntry("panda_right_link1" ,"panda_right_link2" , true);
    acm_->setEntry("panda_right_link1" ,"panda_right_link3" , true);
    acm_->setEntry("panda_right_link1" ,"panda_right_link4" , true);
    acm_->setEntry("panda_right_link2" ,"panda_right_link3" , true);
    acm_->setEntry("panda_right_link2" ,"panda_right_link4" , true);
    acm_->setEntry("panda_right_link3" ,"panda_right_link4" , true);
    acm_->setEntry("panda_right_link3" ,"panda_right_link5" , true);
    acm_->setEntry("panda_right_link3" ,"panda_right_link6" , true);
    acm_->setEntry("panda_right_link3" ,"panda_right_link7" , true);
    acm_->setEntry("panda_right_link3" ,"panda_right_rightfinger" , true);
    acm_->setEntry("panda_right_link4" ,"panda_right_link5" , true);
    acm_->setEntry("panda_right_link4" ,"panda_right_link6" , true);
    acm_->setEntry("panda_right_link4" ,"panda_right_link7" , true);
    acm_->setEntry("panda_right_link4" ,"panda_right_rightfinger" , true);
    acm_->setEntry("panda_right_link5" ,"panda_right_link6" , true);
    acm_->setEntry("panda_right_link5" ,"panda_right_link7" , true);
    acm_->setEntry("panda_right_link6" ,"panda_right_link7" , true);
    acm_->setEntry("panda_right_link6" ,"panda_right_rightfinger" , true);
    acm_->setEntry("panda_right_link7" ,"panda_right_rightfinger" , true);

    crobot_.reset(new DefaultCRobotType(robot_model_));
    cworld_.reset(new DefaultCWorldType());
    robot_state_.reset(new robot_state::RobotState(robot_model_));
    setToHome(*robot_state_);

    base_.reset(new DefaultCWorldType());

    shapes::Shape* box = new shapes::Box(2.4, 1.2, 0.6);
    shapes::ShapeConstPtr box_ptr(box);
    Eigen::Isometry3d box_pos{Eigen::Isometry3d::Identity()};
    box_pos.translation().x() = 1.0;
    box_pos.translation().y() = 0.0;
    box_pos.translation().z() = 0.2; // z : 0.27 됨!
    base_->getWorld()->addToObject("box", box_ptr, box_pos);

  }

public:
  bool robot_model_ok_;

  robot_model::RobotModelPtr robot_model_;
  collision_detection::CollisionRobotPtr crobot_;
  collision_detection::CollisionWorldPtr cworld_;
  collision_detection::CollisionWorldPtr base_;
  collision_detection::AllowedCollisionMatrixPtr acm_;

  robot_state::RobotStatePtr robot_state_;

};

class KinematicChainValidityChecker : public ompl::base::StateValidityChecker // to find valid state space configurations
{
public:
    PandaCollisionCheck collision_checker;
    shapes::ShapeConstPtr shape_ptr;
    Affine3d obj_Sgrasp, Lgrasp_obj;
    KinematicChainValidityChecker(const ompl::base::SpaceInformationPtr &si) : ompl::base::StateValidityChecker(si)
    {
        shapes::Mesh* stefan = shapes::createMeshFromResource("file:///home/jiyeong/catkin_ws/src/1_assembly/grasping_point/STEFAN/stl/assembly_fcl2.stl");
        shape_ptr = shapes::ShapeConstPtr(stefan);

        Vector3d z_offset(0, 0, -0.109);

        Quaterniond q_Lgrasp(0.48089 , 0.518406, -0.518406, 0.48089);
        obj_Sgrasp.linear() = Quaterniond(0.48089 , 0.518406, -0.518406, 0.48089).toRotationMatrix();
        obj_Sgrasp.translation() = Vector3d(-0.417291983962059, 0.385170965965183, 0.189059236695616) + obj_Sgrasp.linear() * z_offset;

        Lgrasp_obj = obj_Sgrasp.inverse();

    }

    bool isValid(const ompl::base::State *state) const override
    {
        const auto *s = state->as<KinematicChainSpace::StateType>();
        // auto &&s = state->as<ompl::base::ConstrainedStateSpace::StateType>()->getState()->as<KinematicChainSpace::StateType>();

        // const auto *s = state->as<ompl::base::RealVectorStateSpace::StateType>();
        
        return isValidImpl(s);
    }

protected:
    bool isValidImpl(const KinematicChainSpace::StateType *s) const
    {
        robot_state::RobotState state1(collision_checker.robot_model_);
        for (int i = 0; i < 14; i++)
            state1.setJointPositions(panda_joint_names[i], &s->values[i]);
        state1.update();

        return !ObjectWorldCollision(&state1) && !selfCollisionCheck(&state1);
        // return !selfCollisionCheck(&state1);
    }

    bool selfCollisionCheck(const robot_state::RobotState *state) const
    {   
        collision_detection::CollisionRequest req;
        collision_detection::CollisionResult res;
        collision_checker.crobot_->checkSelfCollision(req, res, *state, *collision_checker.acm_);
        
        /* COLLISION : TRUE, SAFE : FALSE*/
        // if (res.collision)
        //     std::cout << "robot collision" << std::endl;
        return res.collision;
    }

    bool ObjectWorldCollision(const robot_state::RobotState *state) const
    {   
        // std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        collision_detection::CollisionRequest req;
        collision_detection::CollisionResult res;
        req.max_contacts = 10;
        req.contacts = true;
        req.verbose = false;

        Eigen::Affine3d base_Lgrasp, object_pos;
        base_Lgrasp = state->getFrameTransform("panda_left_hand"); // base to panda_left_hand
        object_pos = base_Lgrasp * Lgrasp_obj;
        Eigen::Isometry3d pos1;
        pos1.translation().x() = object_pos.translation()[0];
        pos1.translation().y() = object_pos.translation()[1];
        pos1.translation().z() = object_pos.translation()[2];
        pos1.linear() = object_pos.linear();
        
        // collision_checker.acm_->setEntry("panda_right_rightfinger", "stefan", true);
        // collision_checker.acm_->setEntry("panda_right_leftfinger", "stefan", true);
        // collision_checker.acm_->setEntry("panda_left_rightfinger", "stefan", true);
        // collision_checker.acm_->setEntry("panda_left_leftfinger", "stefan", true);
        collision_checker.cworld_->getWorld()->addToObject("stefan", shape_ptr, pos1);
        collision_checker.cworld_->checkWorldCollision(req, res, *collision_checker.base_);
        
        // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        // std::cout << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() << " s" << std::endl;

        /* COLLISION : TRUE, SAFE : FALSE*/
        if (res.collision)
        {
            std::cout << object_pos.translation().transpose() << std::endl;
            std::cout << object_pos.linear() << std::endl;
        }
        return res.collision;
    }
};
