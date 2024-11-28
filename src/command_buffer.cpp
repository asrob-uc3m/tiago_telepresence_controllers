#include "command_buffer.hpp"

#include <iterator> // std::advance
#include <ros/ros.h>

constexpr auto CAPACITY_MULTIPLIER = 5;

// -----------------------------------------------------------------------------

void CommandBuffer::accept(const std::vector<double> & command, const ros::SteadyTime & timestamp)
{
    ROS_INFO("in accept()");
    buffer.emplace_back(command, timestamp);

    if (buffer.size() > minSize * (CAPACITY_MULTIPLIER + 1))
    {
        if (left == buffer.begin())
        {
            // undesirable, but we need to keep our iterators valid
            ROS_INFO("advancing left and right in accept()");
            std::advance(left, 1);
            std::advance(right, 1);
        }

        ROS_INFO("popping");
        buffer.pop_front();
        ROS_INFO("popped");

        if (!enabled)
        {
            ROS_INFO("enabling");

            if (left == right)
            {
                ROS_INFO("advancing right in accept()");
                std::advance(right, 1);
                left->second = right->second;
            }

            updateSlopes();
            offset = ros::SteadyTime::now() - left->second;
            enabled = true;
            ROS_INFO("enabled");
        }
    }
}

// -----------------------------------------------------------------------------

void CommandBuffer::updateSlopes()
{
    ROS_INFO("in updateSlopes");
    const auto dt = (right->second - left->second).toSec();

    for (auto i = 0; i < slopes.size(); i++)
    {
        slopes[i] = dt != 0.0 ? (right->first[i] - left->first[i]) / dt : 0.0;
    }
}

// -----------------------------------------------------------------------------

std::vector<double> CommandBuffer::interpolate()
{
    ROS_INFO("in interpolate()");
    const auto refTime = ros::SteadyTime::now() - offset;

    if (enabled && !buffer.empty() && left->second <= refTime)
    {
        ROS_INFO("in interpolate() and doing stuff");
        bool needsUpdate = false;

        while (right->second < refTime && right != buffer.end())
        {
            ROS_INFO("advancing left and right in interpolate()");
            std::advance(left, 1);
            std::advance(right, 1);
            needsUpdate = true;
        }

        ROS_INFO("advanced");

        if (right != buffer.end())
        {
            if (needsUpdate)
            {
                ROS_INFO("updating slopes");
                updateSlopes();
            }

            ROS_INFO("interpolating");
            const auto T = (refTime - left->second).toSec();
            auto out = left->first;

            for (auto i = 0; i < slopes.size(); i++)
            {
                // having y = f(t): f(t+T) = f(t) + T * (delta_y / delta_t)
                out[i] += T * slopes[i];
            }

            ROS_INFO("interpolated");
            return out;
        }
        else
        {
            ROS_INFO("not interpolating");
            right = left;
        }
    }

    enabled = false;

    ROS_INFO("returning");
    return right->first;
}

// -----------------------------------------------------------------------------

ros::WallDuration CommandBuffer::getCommandPeriod() const
{
    return !buffer.empty() ? right->second - left->second : ros::WallDuration(0.0);
}

// -----------------------------------------------------------------------------

void CommandBuffer::reset(const std::vector<double> & initialCommand)
{
    offset.fromSec(0.0);

    buffer.clear();
    slopes.clear();

    buffer.resize(minSize * CAPACITY_MULTIPLIER, std::make_pair(initialCommand, ros::SteadyTime(0.0)));
    slopes.resize(initialCommand.size(), 0.0);

    left = right = buffer.end();
    std::advance(left, -1);
    std::advance(right, -1);

    enabled = false;
}

// -----------------------------------------------------------------------------
