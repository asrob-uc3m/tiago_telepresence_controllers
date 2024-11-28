#include "command_buffer.hpp"

#include <iterator> // std::advance
#include <ros/ros.h>

constexpr auto CAPACITY_MULTIPLIER = 50;

// -----------------------------------------------------------------------------

void CommandBuffer::accept(const std::vector<double> & command, const ros::SteadyTime & timestamp)
{
    ROS_INFO("%s: in accept() %d", name.c_str(), buffer.size());
    buffer.emplace_back(command, timestamp);
    ROS_INFO("%s: accepted %d", name.c_str(), buffer.size());

    if (buffer.size() > minSize * (CAPACITY_MULTIPLIER + 1))
    {
        if (left == buffer.begin())
        {
            // undesirable, but we need to keep our iterators valid
            ROS_INFO("%s: advancing left and right in accept()", name.c_str());
            std::advance(left, 1);
            std::advance(right, 1);
        }

        ROS_INFO("%s: popping", name.c_str());
        buffer.pop_front();
        ROS_INFO("%s: popped", name.c_str());

        if (!enabled)
        {
            ROS_INFO("%s: enabling", name.c_str());

            if (left == right)
            {
                ROS_INFO("%s: advancing right in accept()", name.c_str());
                std::advance(right, 1);
                left->second = right->second;
            }

            updateSlopes();
            offset = ros::SteadyTime::now() - left->second;
            enabled = true;
            ROS_INFO("%s: enabled", name.c_str());
        }
    }

    ROS_INFO("%s: returning from accept()", name.c_str());
}

// -----------------------------------------------------------------------------

void CommandBuffer::updateSlopes()
{
    ROS_INFO("%s: in updateSlopes", name.c_str());
    const auto dt = (right->second - left->second).toSec();

    for (auto i = 0; i < slopes.size(); i++)
    {
        slopes[i] = dt != 0.0 ? (right->first[i] - left->first[i]) / dt : 0.0;
    }
}

// -----------------------------------------------------------------------------

std::vector<double> CommandBuffer::interpolate()
{
    ROS_INFO("%s: in interpolate()", name.c_str());
    const auto refTime = ros::SteadyTime::now() - offset;

    if (enabled && !buffer.empty() && left->second <= refTime)
    {
        ROS_INFO("%s: in interpolate() and doing stuff", name.c_str());
        bool needsUpdate = false;

        while (right->second < refTime && right != buffer.end())
        {
            ROS_INFO("%s: advancing left and right in interpolate()", name.c_str());
            std::advance(left, 1);
            std::advance(right, 1);
            needsUpdate = true;
        }

        ROS_INFO("%s: advanced", name.c_str());

        if (right != buffer.end())
        {
            if (needsUpdate)
            {
                ROS_INFO("%s: updating slopes", name.c_str());
                updateSlopes();
            }

            ROS_INFO("%s: interpolating", name.c_str());
            const auto T = (refTime - left->second).toSec();
            auto out = left->first;

            for (auto i = 0; i < slopes.size(); i++)
            {
                // having y = f(t): f(t+T) = f(t) + T * (delta_y / delta_t)
                out[i] += T * slopes[i];
            }

            ROS_INFO("%s: interpolated", name.c_str());
            return out;
        }
        else
        {
            ROS_INFO("%s: not interpolating", name.c_str());
            right = left;
        }
    }

    enabled = false;

    ROS_INFO("%s: returning", name.c_str());
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
    ROS_INFO("%s: in reset()", name.c_str());
    offset.fromSec(0.0);

    ROS_INFO("%s: clearing buffer", name.c_str());
    buffer.clear();
    slopes.clear();

    ROS_INFO("%s: resizing buffer", name.c_str());
    buffer.resize(minSize * CAPACITY_MULTIPLIER, std::make_pair(initialCommand, ros::SteadyTime(0.0)));
    slopes.resize(initialCommand.size(), 0.0);

    ROS_INFO("%s: setting left and right", name.c_str());
    left = right = buffer.end();
    std::advance(left, -1);
    std::advance(right, -1);

    ROS_INFO("%s: disabling", name.c_str());
    enabled = false;
}

// -----------------------------------------------------------------------------
