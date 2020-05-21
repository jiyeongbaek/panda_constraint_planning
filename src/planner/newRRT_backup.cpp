

#include <constraint_planner/planner/newRRT.h>
#include <limits>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/tools/config/SelfConfig.h>

ompl::geometric::newRRT::newRRT(const base::SpaceInformationPtr &si, bool addIntermediateStates)
  : base::Planner(si, addIntermediateStates ? "newRRTintermediate" : "newRRT")
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

    Planner::declareParam<double>("range", this, &newRRT::setRange, &newRRT::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &newRRT::setGoalBias, &newRRT::getGoalBias, "0.:.05:1.");

}

ompl::geometric::newRRT::~newRRT()
{
    freeMemory();
}

void ompl::geometric::newRRT::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = nullptr;
}

void ompl::geometric::newRRT::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);
    // maxDistance_ = 1.5;
    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
}

void ompl::geometric::newRRT::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }
}

ompl::base::PlannerStatus ompl::geometric::newRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    // auto *start_state = pdef_->getStartState(0)->as<base::ConstrainedStateSpace::StateType>()->getState()->as<KinematicChainSpace::StateType>();
    // auto *goal_state = goal->as<base::GoalState>()->getState()->as<base::ConstrainedStateSpace::StateType>()->getState()->as<KinematicChainSpace::StateType>();

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        nn_->add(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;
    base::State *xstate = si_->allocState();
    
    // base::State *mid = si_->allocState();
    // Eigen::VectorXd start_state_ =  Eigen::Map<const Eigen::VectorXd>(start_state->values, 14);
    // // Eigen::VectorXd goal_state_ =  Eigen::Map<const Eigen::VectorXd>(goal_state->values, 14);
    // for (int i = 0; i < 14; i++)      
    //     mid->as<base::ConstrainedStateSpace::StateType>()->getState()->as<KinematicChainSpace::StateType>()->values[i] = (start_state_[i] + goal_state->values[i] ) /2.0;
    
    // double state_norm = (goal_state_ - start_state_).norm();
    // double delta = state_norm / 7;
    while (!ptc)
    {  
        /* sample random state (with goal biasing) */
        if ((goal_s != nullptr) && rng_.uniform01() < goalBias_ && goal_s->canSample())
        {
            goal_s->sampleGoal(rstate);
        }
        else
            sampler_->sampleUniform(rstate);

        /* find closest state in the tree */
        Motion *nmotion = nn_->nearest(rmotion);
        base::State *dstate = rstate;
        
        /* find state to add */
        double d = si_->distance(nmotion->state, rstate);
        if (d > maxDistance_)
        {
            si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
            // si_->printState(xstate, std::cout);
            dstate = xstate;
        }

        if (si_->checkMotion(nmotion->state, dstate))
        {
            // std::cout << "check motion" << std::endl;
            Motion *motion = new Motion(si_);
            // si_->copyState(motion->state, dstate);
            motion->parent = nmotion;
            nn_->add(motion);
            nmotion = motion;
            

            double dist = 0.0;
            bool sat = goal->isSatisfied(nmotion->state, &dist);
            if (sat)
            {
                approxdif = dist;
                solution = nmotion;
                break;
            }
            if (dist < approxdif)
            {
                approxdif = dist;
                approxsol = nmotion;
            }
        }
    }

    bool solved = false;
    bool approximate = false;
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    }

    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;

        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
        solved = true;
    }

    si_->freeState(xstate);
    if (rmotion->state != nullptr)
        si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());
    // std::vector<Motion *> motions;
    // nn_->list(motions);
    // for (auto &motion : motions)
    // {
    //     si_->printState(motion->state);
    // }
    return base::PlannerStatus(solved, approximate);
}

void ompl::geometric::newRRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    if (lastGoalMotion_ != nullptr)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state));
    }
}
