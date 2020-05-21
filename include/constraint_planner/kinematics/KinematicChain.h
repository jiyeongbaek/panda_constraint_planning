#pragma once

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <boost/math/constants/constants.hpp>
#include <boost/format.hpp>
#include <fstream>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <chrono>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
// #include <moveit/kinematic_constraints/utils.h>
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

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/collision_detection/collision_tools.h>

#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>

#include <constraint_planner/kinematics/grasping point.h>
// #include <constraint_planner/base/jy_ConstrainedValidStateSampler.h>
#include <ompl/base/ConstrainedSpaceInformation.h>

#include <mutex>

namespace ob = ompl::base;
using namespace Eigen;

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
        std::cout << "  - min: ";
        for (unsigned int i = 0; i < dimension_; ++i)
            std::cout << bounds.low[i] << " ";
        std::cout << std::endl;
        std::cout << "  - max: ";
        for (unsigned int i = 0; i < dimension_; ++i)
            std::cout << bounds.high[i] << "  ";
        std::cout << std::endl;

        // type_ = ompl::base::STATE_SPACE_SO2;
        type_ = ompl::base::STATE_SPACE_REAL_VECTOR;
    }

    void enforceBounds(ompl::base::State *state) const override
    {
        auto *rstate = static_cast<StateType *>(state);
        for (unsigned int i = 0; i < dimension_; ++i)
        {
            if (rstate->values[i] > bounds_.high[i])
            {
                std::cout << "enforce bounds" << std::endl;
                rstate->values[i] = bounds_.high[i];
            }
            else if (rstate->values[i] < bounds_.low[i])
            {
                std::cout << "enforce bounds" << std::endl;
                rstate->values[i] = bounds_.low[i];   
            }
        }
    }



    // void registerProjections() override
    // {
    //     registerDefaultProjection(std::make_shared<KinematicChainProjector>(this));
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

class KinematicChainValidityChecker : public ompl::base::StateValidityChecker // to find valid state space configurations
{
public:
    KinematicChainValidityChecker(const ompl::base::SpaceInformationPtr &si) : ompl::base::StateValidityChecker(si)
    {
        robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
        robot_model = robot_model_loader.getModel();
        planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
        acm_ = std::make_shared<collision_detection::AllowedCollisionMatrix>(planning_scene->getAllowedCollisionMatrix());
        robot_state::RobotState &current_state = planning_scene->getCurrentStateNonConst();
        
        Eigen::VectorXd default_start(14);
        default_start << 0, -0.785, 0, -1.571, 0, 1.571, 0.785;
        current_state.setJointGroupPositions("panda_1", default_start);
        current_state.setJointGroupPositions("panda_2", default_start);
        current_state.setJointGroupPositions("panda_3", default_start);
        current_state.update();

        planning_group = robot_model->getJointModelGroup(grp.planning_group);

        std::vector<double> start_values;
        start_values.resize(grp.start.size());
        VectorXd::Map(&start_values[0], grp.start.size()) = grp.start;
        current_state.setJointGroupPositions(planning_group, grp.start);
        current_state.update();

        moveit_msgs::AttachedCollisionObject stefan_obj;
        stefan_obj.link_name = "panda_3_hand";
        stefan_obj.object.header.frame_id = "panda_3_hand";
        stefan_obj.object.id = "stefan"; 
        // stefan_obj.touch_links = {"hand_closed_chain", "panda_1_leftfinger", "panda_1_rightfinger", "panda_3_leftfinger", "panda_3_rightfinger"};
        stefan_obj.touch_links = robot_model->getJointModelGroup(grp.hand_group)->getLinkModelNames();
        shapes::Mesh *m = shapes::createMeshFromResource("file:///home/jiyeong/catkin_ws/src/1_assembly/grasping_point/STEFAN/stl/assembly.stl");
        shape_msgs::Mesh stefan_mesh;
        shapes::ShapeMsg stefan_mesh_msg;
        shapes::constructMsgFromShape(m, stefan_mesh_msg);
        stefan_mesh = boost::get<shape_msgs::Mesh>(stefan_mesh_msg);

        geometry_msgs::Pose stefan_pose;
        
        Eigen::Quaterniond quat(grp.Mgrp_obj.linear());
        stefan_pose.orientation.x = quat.x();
        stefan_pose.orientation.y = quat.y();
        stefan_pose.orientation.z = quat.z();
        stefan_pose.orientation.w = quat.w();

        stefan_pose.position.x = grp.Mgrp_obj.translation()[0];
        stefan_pose.position.y = grp.Mgrp_obj.translation()[1];
        stefan_pose.position.z = grp.Mgrp_obj.translation()[2];

        stefan_obj.object.meshes.push_back(stefan_mesh);
        stefan_obj.object.mesh_poses.push_back(stefan_pose);
        stefan_obj.object.operation = stefan_obj.object.ADD;

        moveit_scene.robot_state.attached_collision_objects.push_back(stefan_obj);
        moveit_scene.is_diff = true;
        planning_scene->processAttachedCollisionObjectMsg(stefan_obj);
    }

    // bool isValid(const ompl::base::State *state) const override
    protected:
    bool isValidImpl(const KinematicChainSpace::StateType *state) const 
    {
        // auto &&s = state->as<ompl::base::ConstrainedStateSpace::StateType>()->getState()->as<KinematicChainSpace::StateType>();
        // const auto *s = state->as<KinematicChainSpace::StateType>();
        collision_detection::CollisionRequest req;
        collision_detection::CollisionResult res;
        bool collision;
        res.clear();
        {
            std::lock_guard<std::mutex> lg(locker_);
            robot_state::RobotState robot_state = planning_scene->getCurrentState();
            robot_state.setJointGroupPositions(planning_group, state->values);
            robot_state.update();
            collision = planning_scene->isStateColliding(robot_state, grp.planning_group);
            // planning_scene->checkCollision(req, res, robot_state);
        }

        // return !res.collision;
        return !collision;

        // req.contacts = true;
        // req.max_contacts = 100;

        // for (int i = 0; i < 14; i++)
        //     robot_state.setJointPositions(panda_joint_names[i], &s->values[i]);

        // bool collision = planning_scene->isStateColliding(robot_state);
        // return !collision;

        // collision_detection::CollisionResult::ContactMap::const_iterator it;
        // for (it = res.contacts.begin(); it != res.contacts.end(); ++it)
        // {
        //     ROS_INFO("Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());
        // }
    }

private:
    robot_model::RobotModelPtr robot_model;
    std::shared_ptr<planning_scene::PlanningScene> planning_scene;
    collision_detection::AllowedCollisionMatrixPtr acm_;
    moveit_msgs::PlanningScene moveit_scene;
    grasping_point grp;
    robot_state::JointModelGroup* planning_group;
protected:
    mutable std::mutex locker_;
};
