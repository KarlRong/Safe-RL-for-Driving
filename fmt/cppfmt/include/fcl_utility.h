#pragma once

#include "fcl/fcl.h"

/// @brief Collision data stores the collision request and the result given by collision algorithm.
template <typename S>
struct CollisionData
{
    CollisionData()
    {
        done = false;
    }

    /// @brief Collision request
    fcl::CollisionRequest<S> request;

    /// @brief Collision result
    fcl::CollisionResult<S> result;

    /// @brief Whether the collision iteration can stop
    bool done;
};

//==============================================================================
template <typename S>
bool defaultCollisionFunction(fcl::CollisionObject<S> *o1, fcl::CollisionObject<S> *o2, void *cdata_)
{
    auto *cdata = static_cast<CollisionData<S> *>(cdata_);
    const auto &request = cdata->request;
    auto &result = cdata->result;

    if (cdata->done)
        return true;

    collide(o1, o2, request, result);

    if (!request.enable_cost && (result.isCollision()) && (result.numContacts() >= request.num_max_contacts))
        cdata->done = true;

    return cdata->done;
}