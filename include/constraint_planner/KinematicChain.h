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

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>

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

    // void registerProjections() override
    // {
    //     registerDefaultProjection(std::make_shared<KinematicChainProjector>(this));
    // }


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


protected:
   
};

#include <moveit/collision_detection/collision_common.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/utils/robot_model_test_utils.h>

#include <urdf_parser/urdf_parser.h>

#include <sstream>
#include <algorithm>
#include <ctype.h>
#include <fstream>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/shape_operations.h>

const std::string panda_joint_names[14] = {"panda_left_joint1", "panda_left_joint2", "panda_left_joint3", "panda_left_joint4", "panda_left_joint5", "panda_left_joint6", "panda_left_joint7",
                            "panda_right_joint1", "panda_right_joint2", "panda_right_joint3", "panda_right_joint4", "panda_right_joint5", "panda_right_joint6", "panda_right_joint7"};

class PandaCollisionCheck
{
public:
  PandaCollisionCheck()
  {
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    kinematic_model = robot_model_loader.getModel();
    planning_scene = std::make_shared<planning_scene::PlanningScene> (kinematic_model);
    acm_ = std::make_shared<collision_detection::AllowedCollisionMatrix> (planning_scene->getAllowedCollisionMatrix());

    moveit_msgs::AttachedCollisionObject stefan_obj;
    stefan_obj.link_name = "panda_left_hand";
    stefan_obj.object.header.frame_id = "panda_left_hand";
    stefan_obj.object.id = "stefan"; /* The id of the object is used to identify it. */
    
    shapes::Mesh* m = shapes::createMeshFromResource("file:///home/jiyeong/catkin_ws/src/1_assembly/grasping_point/STEFAN/stl/assembly.stl");
    shape_msgs::Mesh stefan_mesh;
    shapes::ShapeMsg stefan_mesh_msg;
    shapes::constructMsgFromShape(m, stefan_mesh_msg);
    stefan_mesh = boost::get<shape_msgs::Mesh>(stefan_mesh_msg);
    
    // 파지점 
    Vector3d z_offset(0, 0, -0.109);
    Eigen::Affine3d obj_Lgrasp, Lgrasp_obj;
    obj_Lgrasp.linear() = Quaterniond(0.48089 , 0.518406, -0.518406, 0.48089).toRotationMatrix();
    obj_Lgrasp.translation() = Vector3d(-0.417291983962059, 0.385170965965183, 0.189059236695616) + obj_Lgrasp.linear() * z_offset;
    Lgrasp_obj = obj_Lgrasp.inverse();
    Quaterniond orientation(Lgrasp_obj.linear());

    geometry_msgs::Pose stefan_pose;
    stefan_pose.orientation.w = orientation.w();
    stefan_pose.orientation.x = orientation.x();
    stefan_pose.orientation.y = orientation.y();
    stefan_pose.orientation.z = orientation.z();
    stefan_pose.position.x = Lgrasp_obj.translation()[0];
    stefan_pose.position.y = Lgrasp_obj.translation()[1];
    stefan_pose.position.z = Lgrasp_obj.translation()[2];
    
    stefan_obj.object.meshes.push_back(stefan_mesh);
    stefan_obj.object.mesh_poses.push_back(stefan_pose);
    stefan_obj.object.operation = stefan_obj.object.ADD;

    // moveit_scene = planning_scene->getPlanningSceneMsg();
    moveit_scene.robot_state.attached_collision_objects.push_back(stefan_obj);
    moveit_scene.is_diff = true;
  }

public:
  bool robot_model_ok_;
  robot_model_loader::RobotModelLoader robot_model_loader;    
  robot_model::RobotModelPtr kinematic_model;
    std::shared_ptr<planning_scene::PlanningScene> planning_scene;
    collision_detection::AllowedCollisionMatrixPtr acm_;
    moveit_msgs::PlanningScene moveit_scene;
};

class KinematicChainValidityChecker : public ompl::base::StateValidityChecker // to find valid state space configurations
{
public:
    PandaCollisionCheck collision_checker;
    KinematicChainValidityChecker(const ompl::base::SpaceInformationPtr &si) : ompl::base::StateValidityChecker(si){    }
    bool isValid(const ompl::base::State *state) const override
    {
        const KinematicChainSpace *space = si_->getStateSpace()->as<KinematicChainSpace>();
        const auto *s = state->as<KinematicChainSpace::StateType>();
        
        collision_detection::CollisionRequest req;
        collision_detection::CollisionResult res;
        // req.contacts = true;
        // req.max_contacts = 1000;

        robot_state::RobotState robot_state = collision_checker.planning_scene->getCurrentState();
        // or robot_state::RobotState robot_state(collision_checker.kinematic_model);
        for (int i = 0; i < 14; i++)
            robot_state.setJointPositions(panda_joint_names[i], &s->values[i]);
        robot_state.update();
        collision_checker.planning_scene->setCurrentState(robot_state);

        collision_checker.acm_->setEntry("panda_right_rightfinger", "stefan", true);
        collision_checker.acm_->setEntry("panda_right_leftfinger", "stefan", true);
        collision_checker.acm_->setEntry("panda_left_rightfinger", "stefan", true);
        collision_checker.acm_->setEntry("panda_left_leftfinger", "stefan", true);
        collision_checker.planning_scene->setPlanningSceneMsg(collision_checker.moveit_scene);
        // collision_checker.planning_scene->checkCollision(req, res, robot_state, *collision_checker.acm_);
        collision_checker.planning_scene->checkCollision(req, res);
        return !res.collision;
    }
};

// class ConstrainedKinematicChainValidityChecker : public KinematicChainValidityChecker
// {
// public:
//     ConstrainedKinematicChainValidityChecker(const ob::jy_ConstrainedSpaceInformationPtr &si)
//         : KinematicChainValidityChecker(si)
//     {
//     }
//     bool isValid(const ob::State *state) const override
//     {
//         auto &&space = si_->getStateSpace()->as<ob::ConstrainedStateSpace>()->getSpace()->as<KinematicChainSpace>();
//         auto &&s = state->as<ob::ConstrainedStateSpace::StateType>()->getState()->as<KinematicChainSpace::StateType>();
//         // if (isValidImpl(space, s))
//         //     OMPL_INFORM("FOUND VALID STATE");
//         return isValidImpl(space, s);
//     }
// };
