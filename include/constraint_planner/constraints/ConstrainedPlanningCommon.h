#pragma once

#include <iostream>
#include <fstream>

#include <boost/format.hpp>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>

#include <constraint_planner/constraints/ConstraintFunction.h>
#include <constraint_planner/base/jy_ConstrainedValidStateSampler.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>
// #include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <constraint_planner/base/jy_ProjectedStateSpace.h>

#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <constraint_planner/planner/newRRT.h>
#include <constraint_planner/planner/newPRM.h>
#include <constraint_planner/planner/newRRTConnect.h>
#include <ompl/tools/benchmark/Benchmark.h>
// #include <ompl/base/goals/GoalLazySamples.h>
#include <constraint_planner/base/jy_GoalLazySamples.h>

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
    newPRM,
    newRRTConnect
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
    else if (token == "newRRTConnect")
        type = newRRTConnect;
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
        csi = std::make_shared<ob::ConstrainedSpaceInformation>(css);
        css->setup();
        ss = std::make_shared<og::SimpleSetup>(csi);

        csi->setValidStateSamplerAllocator([](const ob::SpaceInformation *si) -> std::shared_ptr<ob::ValidStateSampler> {
            return std::make_shared<jy_ConstrainedValidStateSampler>(si);
        });

        base_serve = grp.base_serve;
        base_main = grp.base_main;
        obj_Sgrasp = grp.obj_Sgrasp;
        obj_Mgrasp = grp.obj_Mgrasp;
    }

    /* . The distance between each point in the discrete geodesic is tuned by the "delta" parameter
         Valid step size for manifold traversal with delta*/
    void setConstrainedOptions()
    {
        // rrt 되는거
        c_opt.delta = 0.3; //0.075

        c_opt.lambda = 5.0;
        c_opt.tolerance1 = 0.002; //0.001
        c_opt.tolerance2 = 0.025;  // 1degree
        c_opt.time = 90.;
        c_opt.tries = 200;
        // c_opt.range = 1.5;

        constraint->setTolerance(c_opt.tolerance1, c_opt.tolerance2);
        constraint->setMaxIterations(c_opt.tries);

        css->setDelta(c_opt.delta);
        css->setLambda(c_opt.lambda);
    }

    void setStartAndGoalStates()
    {
        ob::ScopedState<> sstart(css);
        ob::ScopedState<> sgoal(css);

        sstart->as<ob::ConstrainedStateSpace::StateType>()->copy(grp.start);
        ob::State *result = csi->allocState();
        bool samplegoal = false;
        do
        {
            samplegoal = sampleIKgoal(result);
        } while (!samplegoal);
        sgoal = result;
        ss->setStartAndGoalStates(sstart, sgoal);
        csi->printState(result);
    }

    ob::PlannerStatus solveOnce(bool goalsampling, const std::string &name = "projection")
    {
        ss->setup();
        ob::jy_GoalSamplingFn samplingFunction = [&](const ob::jy_GoalLazySamples *gls, ob::State *result) {
            return sampleIKgoal(gls, result);
        };

        // ob::jy_GoalSamplingFn start_samplingFunction = [&](const ob::jy_GoalLazySamples *gls, ob::State *result) {
        //     return startsampleIKgoal(gls, result);
        // };

        std::shared_ptr<ompl::base::jy_GoalLazySamples> goal, start;
        // start = std::make_shared<ompl::base::jy_GoalLazySamples>(ss->getSpaceInformation(), start_samplingFunction);

        if (goalsampling)
        {
            goal = std::make_shared<ompl::base::jy_GoalLazySamples>(ss->getSpaceInformation(), samplingFunction, false);
            ob::State *first_goal = csi->allocState();
            if (sampleIKgoal(first_goal));
            {
                csi->printState(first_goal);
                goal->addState(first_goal);
            }
            goal->startSampling();
            ss->setGoal(goal);
        }

        ob::PlannerStatus stat = ss->solve(c_opt.time);
        dumpGraph("test");
        if (stat)
        {
            ompl::geometric::PathGeometric path = ss->getSolutionPath();
            // if (!path.check())
            //     OMPL_WARN("Path fails check!");
            if (stat == ob::PlannerStatus::APPROXIMATE_SOLUTION)
                OMPL_WARN("Solution is approximate.");
            path.printAsMatrix(std::cout);
            OMPL_INFORM("Interpolating path & Dumping path to `%s_path.txt`.", name.c_str());
            path.interpolate();
            std::ofstream pathfile("/home/jiyeong/catkin_ws/" + name + "_path.txt");
            path.printAsMatrix(pathfile);
            pathfile.close();
            std::cout << std::endl;
        }
        else
            OMPL_WARN("No solution found.");
        
        // start->as<ob::jy_GoalLazySamples>()->stopSampling();
        if (goalsampling)
            goal->as<ob::jy_GoalLazySamples>()->stopSampling();

        return stat;
    }

    void dumpGraph(const std::string &name)
    {
        ob::PlannerData data(csi);
        pp->getPlannerData(data);

        std::ofstream graphfile("/home/jiyeong/catkin_ws/" + name + "_path.graphml");
        data.printGraphML(graphfile);
        graphfile.close();

        std::ofstream graphfile2("/home/jiyeong/catkin_ws/" + name + "_path.dot");
        data.printGraphviz(graphfile2);
        graphfile2.close();

        OMPL_INFORM("Dumping planner graph to `%s_graph.graphml`.", name.c_str());
    }

    bool startsampleIKgoal(const ob::jy_GoalLazySamples *gls, ob::State *result)
    {
        std::shared_ptr<panda_ik> panda_ik_solver = std::make_shared<panda_ik>();
        Affine3d base_obj;
        // closed chain
        base_obj.linear().setIdentity();
        base_obj.translation() = Vector3d(1.15, -0.1, 1.0);

        Affine3d target_serve = base_serve.inverse() * base_obj * obj_Sgrasp;
        Affine3d target_main = base_main.inverse() * base_obj * obj_Mgrasp;

        Eigen::Map<Eigen::VectorXd> &sol = *result->as<ob::ConstrainedStateSpace::StateType>();
        int tries = 100;
        while (--tries)
        {
            // std::cout << tries << std::endl;
            bool serve, main;
            serve = panda_ik_solver->randomSolve(target_serve, sol.segment<7>(0));
            main = panda_ik_solver->randomSolve(target_main, sol.segment<7>(7));
            if (serve && main)
            {
                if (gls->getSpaceInformation()->isValid(result))
                {
                    std::cout << "add start state" << std::endl;
                    // csi->printState(result);
                    ss->getProblemDefinition()->addStartState(result);
                    std::this_thread::sleep_for(std::chrono::seconds(10));
                    return true;
                }
            }
        }
        return false;
    }

    bool sampleIKgoal(const ob::jy_GoalLazySamples *gls, ob::State *result)
    {
        int stefan_tries = 500;
        std::shared_ptr<panda_ik> panda_ik_solver = std::make_shared<panda_ik>();
        while (--stefan_tries)
        {
            Affine3d base_obj;
            // closed chain
            base_obj.linear() = AngleAxisd(rng_.uniformReal(M_PI / 2 - deg2rad(2), M_PI / 2 + deg2rad(2)), Eigen::Vector3d::UnitX()) *
                                AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                AngleAxisd(0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            // base_obj.translation() = Vector3d(1.15, 0.0, 1.0);
            base_obj.translation() = Vector3d(1.15, 0.2, 0.85);

            //chair up
            // base_obj.linear().setIdentity();
            // base_obj.translation() = Vector3d(1.15, -0.1, 1.0);

            Affine3d target_serve = base_serve.inverse() * base_obj * obj_Sgrasp;
            Affine3d target_main = base_main.inverse() * base_obj * obj_Mgrasp;

            Eigen::Map<Eigen::VectorXd> &sol = *result->as<ob::ConstrainedStateSpace::StateType>();
            int tries = 50;
            while (--tries)
            {
                // std::cout << tries << std::endl;
                bool serve, main;
                serve = panda_ik_solver->randomSolve(target_serve, sol.segment<7>(0));
                main = panda_ik_solver->randomSolve(target_main, sol.segment<7>(7));
                if (serve && main)
                {
                    if (gls->getSpaceInformation()->isValid(result))
                    {
                        return true;
                    }
                }
                else
                    break;
            }
        }
        return false;
    }

    bool sampleIKgoal(ob::State *result)
    {
        int stefan_tries = 500;
        std::shared_ptr<panda_ik> panda_ik_solver = std::make_shared<panda_ik>();
        while (--stefan_tries)
        {
            Affine3d base_obj;
            // closed chain
            base_obj.linear() = AngleAxisd(rng_.uniformReal(M_PI / 2 - deg2rad(2), M_PI / 2 + deg2rad(2)), Eigen::Vector3d::UnitX()) *
                                AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                AngleAxisd(0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            // base_obj.translation() = Vector3d(1.15, 0.0, 1.0);
            base_obj.translation() = Vector3d(1.15, 0.2, 0.85);

            Affine3d target_serve = base_serve.inverse() * base_obj * obj_Sgrasp;
            Affine3d target_main = base_main.inverse() * base_obj * obj_Mgrasp;

            Eigen::Map<Eigen::VectorXd> &sol = *result->as<ob::ConstrainedStateSpace::StateType>();
            int tries = 50;
            while (--tries)
            {
                // std::cout << tries << std::endl;
                bool serve, main;
                // serve = panda_ik_solver->randomSolve(target_serve, sol.segment<7>(0));
                // main = panda_ik_solver->randomSolve(target_main, sol.segment<7>(7));
                serve = panda_ik_solver->solve(grp.start.segment<7>(0), target_serve, sol.segment<7>(0));
                main = panda_ik_solver->solve(grp.start.segment<7>(7), target_main, sol.segment<7>(7));
                if (serve && main)
                {
                    if (csi->isValid(result))
                    {
                        return true;
                    }
                }
                else
                    break;
            }
        }
        return false;
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
        case newRRTConnect:
            p = createPlannerRange<og::newRRTConnect>();
            break;
        }
        return p;
    }

    void setPlanner(enum PLANNER_TYPE planner, const std::string &projection = "")
    {
        pp = getPlanner(planner, projection);
        ss->setPlanner(pp);
    }

    ob::StateSpacePtr space;
    ChainConstraintPtr constraint;

    ob::ConstrainedStateSpacePtr css;
    // std::shared_ptr<jy_ConstrainedSpaceInformation> csi;
    ob::ConstrainedSpaceInformationPtr csi;
    ob::PlannerPtr pp;
    og::SimpleSetupPtr ss;

    struct ConstrainedOptions c_opt;
    Affine3d obj_Sgrasp, obj_Mgrasp, base_serve, base_main;
    grasping_point grp;

protected:
    ompl::RNG rng_;
};