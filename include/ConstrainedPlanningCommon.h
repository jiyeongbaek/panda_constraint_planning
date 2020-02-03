/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2018, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Zachary Kingston */

#ifndef OMPL_DEMO_CONSTRAINED_COMMON_
#define OMPL_DEMO_CONSTRAINED_COMMON_

#include <iostream>
#include <fstream>

#include <boost/format.hpp>
#include <boost/program_options.hpp>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>

#include <ompl/base/Constraint.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>
#include <ompl/base/spaces/constraint/AtlasStateSpace.h>
#include <ompl/base/spaces/constraint/TangentBundleStateSpace.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>

#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/est/BiEST.h>
#include <ompl/geometric/planners/est/ProjEST.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>

#include <ompl/tools/benchmark/Benchmark.h>

namespace po = boost::program_options;
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace om = ompl::magic;
namespace ot = ompl::tools;

enum SPACE_TYPE
{
    PJ = 0,
    AT = 1,
    TB = 2
};

const std::string spaceStr[3] = {"PJ", "AT", "TB"};

std::istream &operator>>(std::istream &in, enum SPACE_TYPE &type)
{
    std::string token;
    in >> token;
    if (token == "PJ")
        type = PJ;
    else if (token == "AT")
        type = AT;
    else if (token == "TB")
        type = TB;
    else
        in.setstate(std::ios_base::failbit);

    return in;
}

void addSpaceOption(po::options_description &desc, enum SPACE_TYPE *space)
{
    auto space_msg = "Choose which constraint handling methodology to use. One of:\n"
                     "PJ - Projection (Default), "
                     "AT - Atlas, "
                     "TB - Tangent Bundle.";

    desc.add_options()("space,s", po::value<enum SPACE_TYPE>(space), space_msg);
}

enum PLANNER_TYPE
{
    RRT,
    RRT_I,
    RRTConnect,
    RRTConnect_I,
    RRTstar,
    EST,
    BiEST,
    ProjEST,
    BITstar,
    PRM,
    SPARS,
    KPIECE,
    BKPIECE
};

std::istream &operator>>(std::istream &in, enum PLANNER_TYPE &type)
{
    std::string token;
    in >> token;
    if (token == "RRT")
        type = RRT;
    else if (token == "RRT_I")
        type = RRT_I;
    else if (token == "RRTConnect")
        type = RRTConnect;
    else if (token == "RRTConnect_I")
        type = RRTConnect_I;
    else if (token == "RRTstar")
        type = RRTstar;
    else if (token == "EST")
        type = EST;
    else if (token == "BiEST")
        type = BiEST;
    else if (token == "ProjEST")
        type = ProjEST;
    else if (token == "BITstar")
        type = BITstar;
    else if (token == "PRM")
        type = PRM;
    else if (token == "SPARS")
        type = SPARS;
    else if (token == "KPIECE")
        type = KPIECE;
    else if (token == "BKPIECE")
        type = BKPIECE;
    else
        in.setstate(std::ios_base::failbit);

    return in;
}

void addPlannerOption(po::options_description &desc, std::vector<enum PLANNER_TYPE> *planners)
{
    auto planner_msg = "List of which motion planner to use (multiple if benchmarking, one if planning). Choose from:\n"
                       "RRT (Default), RRT_I, RRTConnect, RRTConnect_I, RRTstar, "
                       "EST, BiEST, ProjEST, "
                       "BITstar, "
                       "PRM, SPARS, "
                       "KPIECE, BKPIECE.";

    desc.add_options()("planner,p", po::value<std::vector<enum PLANNER_TYPE>>(planners)->multitoken(), planner_msg);
}

struct ConstrainedOptions
{
    double delta;
    double lambda;
    double tolerance;
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
    ConstrainedProblem(enum SPACE_TYPE type, ob::StateSpacePtr space_, ob::ConstraintPtr constraint_)
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
        c_opt.delta =  0.8; //0.18
        c_opt.lambda = 3.0;
        c_opt.tolerance = 0.02; // 0.05
        c_opt.time = 600.;
        c_opt.tries = 100;
        c_opt.range = 0;
        
        /* POSE CONSTRAINT PARAMETER */
        // c_opt.delta =  0.1;
        // c_opt.lambda = 3;
        // c_opt.tolerance = 1e-3; //1.5
        // c_opt.time = 600.;
        // c_opt.tries = 600;
        // c_opt.range = 0;

        /* ORIENTATION CONSTRAINT PARAMETER */
        // c_opt.delta =  0.5;
        // c_opt.lambda = 3;
        // c_opt.tolerance = 1.2; //1.5
        // c_opt.time = 1000.;
        // c_opt.tries = 500;
        // c_opt.range = 0;

        constraint->setTolerance(c_opt.tolerance);
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

        a_opt.epsilon = 1.0;
        a_opt.rho = 0.03; //::CONSTRAINED_STATE_SPACE_DELTA * om::ATLAS_STATE_SPACE_RHO_MULTIPLIER;
        a_opt.exploration = 0.20; // om::ATLAS_STATE_SPACE_EXPLORATION;
        a_opt.alpha = om::ATLAS_STATE_SPACE_ALPHA;
        a_opt.bias = false;
        a_opt.separate = true; //default : false
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
            case RRT_I:
                p = createPlannerRange<og::RRT>(true);
                break;
            case RRTConnect:
                p = createPlannerRange<og::RRTConnect>();
                break;
            case RRTConnect_I:
                p = createPlannerRange<og::RRTConnect>(true);
                break;
            case RRTstar:
                p = createPlannerRange<og::RRTstar>();
                break;
            case EST:
                p = createPlannerRange<og::EST>();
                break;
            case BiEST:
                p = createPlannerRange<og::BiEST>();
                break;
            case ProjEST:
                p = createPlannerRangeProj<og::ProjEST>(projection);
                break;
            case BITstar:
                p = createPlanner<og::BITstar>();
                break;
            case PRM:
                p = createPlanner<og::PRM>();
                break;
            case SPARS:
                p = createPlanner<og::SPARS>();
                break;
            case KPIECE:
                p = createPlannerRangeProj<og::KPIECE1>(projection);
                break;
            case BKPIECE:
                p = createPlannerRangeProj<og::BKPIECE1>(projection);
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
        std::cout << std::endl;

        if (stat)
        {
            // Get solution and validate
            // auto path = ss->getSolutionPath();
            ompl::geometric::PathGeometric path = ss->getSolutionPath();
            if (!path.check())
                OMPL_WARN("Path fails check!");

            if (stat == ob::PlannerStatus::APPROXIMATE_SOLUTION)
                OMPL_WARN("Solution is approximate.");

            /* Interpolate and validate interpolated solution path.
            Insert a number of states in a path so that the
                path is made up of exactly \e count states. States are
                inserted uniformly (more states on longer
                segments). Changes are performed only if a path has
                less than \e count states.

             void ompl::geometric::PathGeometric::interpolate()
            {
                std::vector<base::State *> newStates;
                const int segments = states_.size() - 1;
            
                for (int i = 0; i < segments; ++i)
                {
                    base::State *s1 = states_[i];
                    base::State *s2 = states_[i + 1];
            
                    newStates.push_back(s1);
                    unsigned int n = si_->getStateSpace()->validSegmentCount(s1, s2);
            
                    std::vector<base::State *> block;
                    si_->getMotionStates(s1, s2, block, n - 1, false, true);
                    newStates.insert(newStates.end(), block.begin(), block.end());
                }
                newStates.push_back(states_[segments]);
                states_.swap(newStates);
            }
            */
            path.interpolate();

            if (output)
            {
                OMPL_INFORM("Dumping path to `%s_path.txt`.", name.c_str());
                std::ofstream pathfile((boost::format("%1%_path.txt") % name).str());
                path.printAsMatrix(pathfile);
                pathfile.close();
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
    ob::ConstraintPtr constraint;

    ob::ConstrainedStateSpacePtr css;
    ob::ConstrainedSpaceInformationPtr csi;

    ob::PlannerPtr pp;

    og::SimpleSetupPtr ss;

    struct ConstrainedOptions c_opt;
    struct AtlasOptions a_opt;

    ot::Benchmark *bench;
    ot::Benchmark::Request request;
};

#endif
