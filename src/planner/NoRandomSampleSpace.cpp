
#include <constraint_planner/planner/NoRandomSampleSpace.h>

#include <Eigen/Core>
#include <utility>

/// NoRandomSampler

/// Public

ompl::base::NoRandomSampler::NoRandomSampler(const NoRandomSampleSpace *space, StateSamplerPtr sampler)
  : WrapperStateSampler(space, std::move(sampler)), constraint_(space->getConstraint())
{
}

void ompl::base::NoRandomSampler::sampleUniform(State *state)
{
    WrapperStateSampler::sampleUniform(state);
    constraint_->project(state);
    space_->enforceBounds(state);
}

void ompl::base::NoRandomSampler::sampleUniformNear(State *state, const State *near, const double distance)
{
    WrapperStateSampler::sampleUniformNear(state, near, distance);
    constraint_->project(state);
    space_->enforceBounds(state);
}

void ompl::base::NoRandomSampler::sampleGaussian(State *state, const State *mean, const double stdDev)
{
    WrapperStateSampler::sampleGaussian(state, mean, stdDev);
    constraint_->project(state);
    space_->enforceBounds(state);
}

/// NoRandomSampleSpace

/// Public

bool ompl::base::NoRandomSampleSpace::discreteGeodesic(const State *from, const State *to, bool interpolate,
                                                       std::vector<State *> *geodesic) const
{
    // Save a copy of the from state.
    if (geodesic != nullptr)
    {
        geodesic->clear();
        geodesic->push_back(cloneState(from));
    }

    const double tolerance = delta_;

    // No need to traverse the manifold if we are already there.
    double dist, step, total = 0;
    if ((dist = distance(from, to)) <= tolerance)
        return true;

    const double max = dist * lambda_;

    auto previous = cloneState(from);
    auto scratch = allocState();

    auto &&svc = si_->getStateValidityChecker();

    do
    {
        WrapperStateSpace::interpolate(previous, to, delta_ / dist, scratch);

        // Project new state onto constraint manifold
        if (!constraint_->project(scratch)                  // not on manifold
            || !(interpolate || svc->isValid(scratch))      // not valid
            || (step = distance(previous, scratch)) > lambda_ * delta_)  // deviated
            break;

        // Check if we have wandered too far
        total += step;
        if (total > max)
            break;

        // Check if we are no closer than before
        const double newDist = distance(scratch, to);
        if (newDist >= dist)
            break;

        dist = newDist;
        copyState(previous, scratch);

        // Store the new state
        if (geodesic != nullptr)
            geodesic->push_back(cloneState(scratch));

    } while (dist >= tolerance);

    freeState(scratch);
    freeState(previous);

    return dist <= tolerance;
}
