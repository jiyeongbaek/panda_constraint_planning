#pragma once

#include <iostream>
#include <fstream>

#include <boost/format.hpp>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>

#include <constraint_planner/constraints/ConstraintFunction.h>
#include <constraint_planner/base/jy_ConstrainedSpaceInformation.h>

#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>
// #include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <constraint_planner/base/jy_ProjectedStateSpace.h>

#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <constraint_planner/planner/newRRT.h>
#include <constraint_planner/planner/newPRM.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/base/goals/GoalLazySamples.h>

#include <ompl/base/spaces/SE3StateSpace.h>
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace om = ompl::magic;
namespace ot = ompl::tools;
using namespace Eigen;
using namespace std;
enum PLANNER_TYPE
{
    RRT,
    RRTConnect,
    PRM,
    newRRT,
    newPRM
};

std::istream &operator>>(std::istream &in, enum PLANNER_TYPE &type)
{
    std::string token;
    in >> token;
    if (token == "RRT")
        type = RRT;
    else if (token == "RRTConnect")
        type = RRTConnect;
    else if (token == "PRM")
        type = PRM;
    else if (token == "newRRT")
        type = newRRT;
    else if (token == "newPRM")
        type = newPRM;
    else
        in.setstate(std::ios_base::failbit);

    return in;
}

struct ConstrainedOptions
{
    double delta;
    double lambda;
    double tolerance1;
    double tolerance2;
    double time;
    unsigned int tries;
    double range;
};

class ConstrainedProblem
{
public:
    ConstrainedProblem(ob::StateSpacePtr space_, ChainConstraintPtr constraint_)
        : space(std::move(space_)), constraint(std::move(constraint_))
    {
        OMPL_INFORM("Using Projection-Based State Space!");
        css = std::make_shared<jy_ProjectedStateSpace>(space, constraint);
        csi = std::make_shared<ob::jy_ConstrainedSpaceInformation>(css);
        css->setup();
        ss = std::make_shared<og::SimpleSetup>(csi);

        base_left.translation() = Vector3d(0, 0.3, 0.6);
        base_right.translation() = Vector3d(0, -0.3, 0.6);
        base_left.linear().setIdentity();
        base_right.linear().setIdentity();
        
        grasping_point grp;
        obj_Lgrasp = grp.obj_Lgrasp;
        obj_Rgrasp = grp.obj_Rgrasp;
    }

    /* . The distance between each point in the discrete geodesic is tuned by the "delta" parameter
         Valid step size for manifold traversal with delta*/
    void setConstrainedOptions()
    {
        // rrt 되는거
        c_opt.delta = 0.1; //0.075

        c_opt.lambda = 2.0;
        c_opt.tolerance1 = 0.007; //0.001
        c_opt.tolerance2 = 0.08;  // 0.01;
        c_opt.time = 60.;
        c_opt.tries = 200;
        c_opt.range = 1.5;

        constraint->setTolerance(c_opt.tolerance1, c_opt.tolerance2);
        constraint->setMaxIterations(c_opt.tries);

        css->setDelta(c_opt.delta);
        css->setLambda(c_opt.lambda);
    }

    void setStartAndGoalStates(const Eigen::Ref<const Eigen::VectorXd> &start,
                               const Eigen::Ref<const Eigen::VectorXd> &goal)
    {
        // Create start and goal states (poles of the sphere)
        ob::ScopedState<> sstart(css);
        ob::ScopedState<> sgoal(css);

        sstart->as<ob::ConstrainedStateSpace::StateType>()->copy(start);
        sgoal->as<ob::ConstrainedStateSpace::StateType>()->copy(goal);
        ss->setStartAndGoalStates(sstart, sgoal);
    }

    template <typename _T>
    std::shared_ptr<_T> createPlanner()
    {
        auto &&planner = std::make_shared<_T>(csi);
        return std::move(planner);
    }

    template <typename _T>
    std::shared_ptr<_T> createPlannerIntermediate()
    {
        auto &&planner = std::make_shared<_T>(csi, true);
        return std::move(planner);
    }

    template <typename _T>
    std::shared_ptr<_T> createPlannerRange()
    {
        auto &&planner = createPlanner<_T>();

        planner->setRange(c_opt.range);

        return std::move(planner);
    }

    template <typename _T>
    std::shared_ptr<_T> createPlannerRange(bool intermediate)
    {
        auto &&planner = createPlannerIntermediate<_T>();

        planner->setRange(c_opt.range);

        return std::move(planner);
    }

    template <typename _T>
    std::shared_ptr<_T> createPlannerRangeProj(const std::string &projection)
    {
        const bool isProj = projection != "";
        auto &&planner = createPlannerRange<_T>();

        if (isProj)
            planner->setProjectionEvaluator(projection);

        return std::move(planner);
    }

    ob::PlannerPtr getPlanner(enum PLANNER_TYPE planner, const std::string &projection = "")
    {
        ob::PlannerPtr p;
        switch (planner)
        {
        case RRT:
            p = createPlannerRange<og::RRT>();
            break;
        case RRTConnect:
            p = createPlannerRange<og::RRTConnect>();
            break;

        case PRM:
            p = createPlanner<og::PRM>();
            break;

        case newRRT:
            p = createPlanner<og::newRRT>();
            break;

        case newPRM:
            p = createPlanner<og::newPRM>();
            break;
        }
        return p;
    }

    void setPlanner(enum PLANNER_TYPE planner, const std::string &projection = "")
    {
        pp = getPlanner(planner, projection);
        ss->setPlanner(pp);
    }
    
    ob::PlannerStatus solveOnce(bool output = false, const std::string &name = "projection")
    {
        ss->setup();
        ob::GoalSamplingFn samplingFunction = [&](const ob::GoalLazySamples *gls, ob::State *result) {
            return sampleIKgoal(gls, result);
        };

        std::shared_ptr<ompl::base::GoalLazySamples> goal;
        goal = std::make_shared<ompl::base::GoalLazySamples>(ss->getSpaceInformation(), samplingFunction);
        goal->addState(ss->getGoal()->as<ob::GoalState>()->getState());
        ss->setGoal(goal);

        ob::PlannerStatus stat = ss->solve(c_opt.time);

        if (stat)
        {
            ompl::geometric::PathGeometric path = ss->getSolutionPath();
            // if (!path.check())
            //     OMPL_WARN("Path fails check!");
            if (stat == ob::PlannerStatus::APPROXIMATE_SOLUTION)
                OMPL_WARN("Solution is approximate.");
            path.printAsMatrix(std::cout);
            path.interpolate();
            if (output)
            {
                // std::ofstream pathfile((boost::format("%1%_path.txt") % name).str()); //, std::ios::app);
                std::ofstream pathfile("/home/jiyeong/catkin_ws/" + name + "_path.txt");
                OMPL_INFORM("Interpolating path & Dumping path to `%s_path.txt`.", name.c_str());
                path.printAsMatrix(pathfile);
                pathfile.close();
                std::cout << std::endl;
                dumpGraph("test");
            }
        }
        else
            OMPL_WARN("No solution found.");

        goal->as<ob::GoalLazySamples>()->stopSampling();
        return stat;
    }

    void dumpGraph(const std::string &name)
    {
        OMPL_INFORM("Dumping planner graph to `%s_graph.graphml`.", name.c_str());
        ob::PlannerData data(csi);
        pp->getPlannerData(data);

        std::ofstream graphfile((boost::format("%1%_graph.graphml") % name).str());
        data.printGraphML(graphfile);
        graphfile.close();

        std::ofstream graphfile2((boost::format("%1%_graph.dot") % name).str());
        data.printGraphviz(graphfile2);
        graphfile2.close();
    }

    bool sampleIKgoal(const ob::GoalLazySamples *gls, ob::State *result)
    {
        auto sr = result->as<ob::ConstrainedStateSpace::StateType>()->as<KinematicChainSpace::StateType>();

        int stefan_tries = 500;
        while (--stefan_tries)
        {
            Affine3d base_obj;
            
            double yaw = rng_.uniformReal(-M_PI / 2, M_PI / 2);
            base_obj.linear() = AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                                AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX()).toRotationMatrix();
            base_obj.translation() = Vector3d(rng_.uniformReal(0.8, 1.1), rng_.uniformReal(-0.1, 0.1), rng_.uniformReal(0.7, 0.8));
                

            Affine3d target_left = base_left.inverse() * base_obj * obj_Lgrasp;
            Affine3d target_right = base_right.inverse() * base_obj * obj_Rgrasp;


            std::shared_ptr<panda_ik> panda_ik_solver = std::make_shared<panda_ik>();
            // Eigen::VectorXd start;
            // auto start_state = ss->getProblemDefinition()->getStartState(0);
            // auto newstart = start_state->as<ob::ConstrainedStateSpace::StateType>()->getState()->as<KinematicChainSpace::StateType>();
            // start = Eigen::Map<const Eigen::VectorXd>(newstart->values, 14);

            // std::cout << base_obj.translation().transpose() << std::endl;
            // std::cout << base_obj.linear() << std::endl;
            // cout << target_left.translation().transpose() << endl;
            // cout << target_right.translation().transpose() << endl;
            Matrix<double, 14, 1> sol;
            int tries = 50;
            while (--tries)
            {
                // std::cout << tries << std::endl;
                bool left, right;
                left = panda_ik_solver->randomSolve(target_left, sol.segment<7>(0));
                right = panda_ik_solver->randomSolve(target_right, sol.segment<7>(7));
                if ( left && right)
                {
                    // Eigen::Map<Eigen::VectorXd>(sr->values, 14) = sol;
                    for (int i = 0; i < 14; i++)
                        result->as<ob::ConstrainedStateSpace::StateType>()->getState()->as<KinematicChainSpace::StateType>()->values[i] = sol[i];
                    if (gls->getSpaceInformation()->isValid(result))
                        return true;
                }
                else
                    break;
            }
        }
        return false;
    }
    ob::StateSpacePtr space;
    ChainConstraintPtr constraint;

    ob::ConstrainedStateSpacePtr css;
    ob::jy_ConstrainedSpaceInformationPtr csi;
    ob::PlannerPtr pp;
    og::SimpleSetupPtr ss;

    struct ConstrainedOptions c_opt;
    Affine3d obj_Lgrasp, obj_Rgrasp, base_left, base_right;

protected:
    ompl::RNG rng_;
};