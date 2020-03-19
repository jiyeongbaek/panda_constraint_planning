#pragma once

#include <iostream>
#include <fstream>

#include <boost/format.hpp>
#include <boost/program_options.hpp>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>

// #include <ompl/base/Constraint.h>
#include <constraint_planner/Constraint.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>
#include <ompl/base/spaces/constraint/AtlasStateSpace.h>
#include <ompl/base/spaces/constraint/TangentBundleStateSpace.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>

#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <constraint_planner/No_RRT.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>

#include <ompl/geometric/planners/rrt/BiTRRT.h>
#include <ompl/tools/benchmark/Benchmark.h>

namespace po = boost::program_options;
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace om = ompl::magic;
namespace ot = ompl::tools;

enum SPACE_TYPE
{
    PJ = 0,
    PJ2 = 1,
    AT = 2,
    TB = 3
};

const std::string spaceStr[4] = {"PJ", "PJ2", "AT", "TB"};

std::istream &operator>>(std::istream &in, enum SPACE_TYPE &type)
{
    std::string token;
    in >> token;
    if (token == "PJ")
        type = PJ;
    else if (token == "AT")
        type = AT;
    else if (token == "PJ2")
        type = PJ2;
    else if (token == "TB")
        type = TB;
    else
        in.setstate(std::ios_base::failbit);

    return in;
}

enum PLANNER_TYPE
{
    RRT,
    RRTConnect,
    PRM,
    No_RRT,
    KPIECE
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
    else if (token == "No_RRT")
        type = No_RRT;

    else if (token == "KPIECE")
        type = KPIECE;

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

struct AtlasOptions
{
    double epsilon;
    double rho;
    double exploration;
    double alpha;
    bool bias;
    bool separate;
    unsigned int charts;
};

class ConstrainedProblem
{
public:
    ConstrainedProblem(enum SPACE_TYPE type, ob::StateSpacePtr space_, Constraint_newPtr constraint_)
      : type(type), space(std::move(space_)), constraint(std::move(constraint_))
    {
        // Combine the ambient state space and the constraint to create the
        // constrained state space.
        switch (type)
        {
            case PJ:
                OMPL_INFORM("Using Projection-Based State Space!");
                css = std::make_shared<ob::ProjectedStateSpace>(space, constraint);
                csi = std::make_shared<ob::ConstrainedSpaceInformation>(css);
                break;
            case PJ2:
                OMPL_INFORM("Using Projection-Based (no random s) State Space!");
                css = std::make_shared<ob::ProjectedStateSpace>(space, constraint);
                csi = std::make_shared<ob::ConstrainedSpaceInformation>(css);
                break;
            case AT:
                OMPL_INFORM("Using Atlas-Based State Space!");
                css = std::make_shared<ob::AtlasStateSpace>(space, constraint);
                csi = std::make_shared<ob::ConstrainedSpaceInformation>(css);
                break;
            case TB:
                OMPL_INFORM("Using Tangent Bundle-Based State Space!");
                css = std::make_shared<ob::TangentBundleStateSpace>(space, constraint);
                csi = std::make_shared<ob::TangentBundleSpaceInformation>(css);
                break;
        }

        css->setup();
        ss = std::make_shared<og::SimpleSetup>(csi);
    }

    /* . The distance between each point in the discrete geodesic is tuned by the "delta" parameter
         Valid step size for manifold traversal with delta*/
    void setConstrainedOptions()
    {   
        // rrt 되는거
        c_opt.delta =  0.075; //0.20
        c_opt.lambda = 2.0;
        c_opt.tolerance1 = 0.001; //0.0025
        c_opt.tolerance2 = 0.01;
        c_opt.time = 30.;
        c_opt.tries = 1000;
        // c_opt.range = 6;

        constraint->setTolerance(c_opt.tolerance1, c_opt.tolerance2);
        constraint->setMaxIterations(c_opt.tries);

        css->setDelta(c_opt.delta);
        css->setLambda(c_opt.lambda);
    }

    void setAtlasOptions()
    {
        /* 
        static const unsigned int ATLAS_STATE_SPACE_SAMPLES = 50;
        static const double ATLAS_STATE_SPACE_EPSILON = 0.05;
        static const double ATLAS_STATE_SPACE_RHO_MULTIPLIER = 5;
        static const double ATLAS_STATE_SPACE_ALPHA = boost::math::constants::pi<double>() / 8.0;
        static const double ATLAS_STATE_SPACE_EXPLORATION = 0.75;
        static const unsigned int ATLAS_STATE_SPACE_MAX_CHARTS_PER_EXTENSION = 200;
        static const double ATLAS_STATE_SPACE_BACKOFF = 0.75;
        */
        //// Consider decreasing rho and/or the exploration paramter if this becomes a problem.
        // OMPL_WARN("ompl::base::AtlasStateSpace::sampleUniform(): "
        //           "Took too long; returning center of a random chart.");
        a_opt.epsilon = 0.1; //1.0
        a_opt.rho = 0.01; //::CONSTRAINED_STATE_SPACE_DELTA * om::ATLAS_STATE_SPACE_RHO_MULTIPLIER; //the maximum radius for which a chart is valid. Default 0.1.
        a_opt.exploration = 0.05; // om::ATLAS_STATE_SPACE_EXPLORATION;
        a_opt.alpha = om::ATLAS_STATE_SPACE_ALPHA;
        a_opt.bias = false;
        a_opt.separate = false; //default : false
        a_opt.charts = om::ATLAS_STATE_SPACE_MAX_CHARTS_PER_EXTENSION;

        if (!(type == AT || type == TB))
            return;

        auto &&atlas = css->as<ob::AtlasStateSpace>();
        atlas->setExploration(a_opt.exploration);
        atlas->setEpsilon(a_opt.epsilon);
        atlas->setRho(a_opt.rho);
        atlas->setAlpha(a_opt.alpha);
        atlas->setMaxChartsPerExtension(a_opt.charts);

        if (a_opt.bias)
            atlas->setBiasFunction([atlas](ompl::base::AtlasChart *c) -> double {
                return (atlas->getChartCount() - c->getNeighborCount()) + 1;
            });

        if (type == AT)
            atlas->setSeparated(!a_opt.separate);

        atlas->setup();
    }

    void setStartAndGoalStates(const Eigen::Ref<const Eigen::VectorXd> &start,
                               const Eigen::Ref<const Eigen::VectorXd> &goal)
    {
        // Create start and goal states (poles of the sphere)
        ob::ScopedState<> sstart(css);
        ob::ScopedState<> sgoal(css);

        sstart->as<ob::ConstrainedStateSpace::StateType>()->copy(start);
        sgoal->as<ob::ConstrainedStateSpace::StateType>()->copy(goal);

        switch (type)
        {
            case AT:
            case TB:
                css->as<ob::AtlasStateSpace>()->anchorChart(sstart.get());
                css->as<ob::AtlasStateSpace>()->anchorChart(sgoal.get());
                break;
            default:
                break;
        }

        // Setup problem
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

        if (c_opt.range == 0)
        {
            if (type == AT || type == TB)
                planner->setRange(css->as<ob::AtlasStateSpace>()->getRho_s());
        }
        else
            planner->setRange(c_opt.range);

        return std::move(planner);
    }

    template <typename _T>
    std::shared_ptr<_T> createPlannerRange(bool intermediate)
    {
        auto &&planner = createPlannerIntermediate<_T>();

        if (c_opt.range == 0)
        {
            if (type == AT || type == TB)
                planner->setRange(css->as<ob::AtlasStateSpace>()->getRho_s());
        }
        else
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

            case No_RRT:
                p = createPlanner<og::No_RRT>();
                break;

            case KPIECE:
                p = createPlannerRangeProj<og::KPIECE1>(projection);
                break;
        }
        return p;
    }

    void setPlanner(enum PLANNER_TYPE planner, const std::string &projection = "")
    {
        pp = getPlanner(planner, projection);
        ss->setPlanner(pp);
    }

    ob::PlannerStatus solveOnce(bool output = false, const std::string &name="projection")
    {
        ss->setup();
        ob::PlannerStatus stat = ss->solve(c_opt.time); 
        if (stat)
        {
            ompl::geometric::PathGeometric path = ss->getSolutionPath();
            // if (!path.check())
            //     OMPL_WARN("Path fails check!");

            if (stat == ob::PlannerStatus::APPROXIMATE_SOLUTION)
                OMPL_WARN("Solution is approximate.");
            // OMPL_INFORM("Interpolating path ... ");
            path.interpolate();

            if (output)
            {                
                // std::ofstream pathfile((boost::format("%1%_path.txt") % name).str()); //, std::ios::app);
                std::ofstream pathfile("/home/jiyeong/catkin_ws/" + name + "_path.txt"); 
                // std::ofstream pathfile("/home/jiyeong/catkin_ws/result_path.txt", std::ios::app); 
                path.printAsMatrix(pathfile);
                OMPL_INFORM("Interpolating path  &  Dumping path to `%s_path.txt`.", name.c_str());
                pathfile.close();
                std::cout << std::endl;
            }
        }
        else
            OMPL_WARN("No solution found.");

        return stat;
    }

    void setupBenchmark(std::vector<enum PLANNER_TYPE> &planners, const std::string &problem)
    {
        bench = new ot::Benchmark(*ss, problem);

        bench->addExperimentParameter("n", "INTEGER", std::to_string(constraint->getAmbientDimension()));
        bench->addExperimentParameter("k", "INTEGER", std::to_string(constraint->getManifoldDimension()));
        bench->addExperimentParameter("n - k", "INTEGER", std::to_string(constraint->getCoDimension()));
        bench->addExperimentParameter("space", "INTEGER", std::to_string(type));
        
        request = ot::Benchmark::Request(c_opt.time, 2048, 100, 0.1, true, false, true, true);
        
        for (auto planner : planners)
            bench->addPlanner(getPlanner(planner, problem));

        bench->setPreRunEvent([&](const ob::PlannerPtr &planner) {
            if (type == AT || type == TB)
                planner->getSpaceInformation()->getStateSpace()->as<ompl::base::AtlasStateSpace>()->clear();
            else
                planner->getSpaceInformation()->getStateSpace()->as<ompl::base::ConstrainedStateSpace>()->clear();

            planner->clear();
        });
    }

    void runBenchmark()
    {
        bench->benchmark(request);

        auto now(ompl::time::as_string(ompl::time::now()));
        const std::string filename =
            (boost::format("%1%_%2%_%3%_benchmark.log") % now % bench->getExperimentName() % spaceStr[type]).str();

        bench->saveResultsToFile(filename.c_str());
    }

    void atlasStats()
    {
        // For atlas types, output information about size of atlas and amount of space explored
        if (type == AT || type == TB)
        {
            auto at = css->as<ob::AtlasStateSpace>();
            OMPL_INFORM("Atlas has %zu charts", at->getChartCount());
            if (type == AT)
                OMPL_INFORM("Atlas is approximately %.3f%% open", at->estimateFrontierPercent());
        }
    }

    void dumpGraph(const std::string &name)
    {
        OMPL_INFORM("Dumping planner graph to `%s_graph.graphml`.", name.c_str());
        ob::PlannerData data(csi);
        pp->getPlannerData(data);

        std::ofstream graphfile((boost::format("%1%_graph.graphml") % name).str());
        data.printGraphML(graphfile);
        graphfile.close();

        if (type == AT || type == TB)
        {
            OMPL_INFORM("Dumping atlas to `%s_atlas.ply`.", name.c_str());
            std::ofstream atlasfile((boost::format("%1%_atlas.ply") % name).str());
            css->as<ob::AtlasStateSpace>()->printPLY(atlasfile);
            atlasfile.close();
        }
    }

    enum SPACE_TYPE type;

    ob::StateSpacePtr space;
    // ob::ConstraintPtr constraint;
    Constraint_newPtr constraint;

    ob::ConstrainedStateSpacePtr css;
    ob::ConstrainedSpaceInformationPtr csi;

    ob::PlannerPtr pp;

    og::SimpleSetupPtr ss;

    struct ConstrainedOptions c_opt;
    struct AtlasOptions a_opt;

    ot::Benchmark *bench;
    ot::Benchmark::Request request;
};

