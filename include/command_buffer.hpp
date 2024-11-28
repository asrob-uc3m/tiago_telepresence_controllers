#ifndef __COMMAND_BUFFER_HPP__
#define __COMMAND_BUFFER_HPP__

#include <list>
#include <utility> // std::pair
#include <vector>
#include <ros/time.h>

class CommandBuffer
{
public:
    CommandBuffer(const std::string & name, int minSize, int cmdSize) : name(name), minSize(minSize), slopes(cmdSize, 0.0) { }
    void accept(const std::vector<double> & command, const ros::SteadyTime & timestamp);
    std::vector<double> interpolate();
    ros::WallDuration getCommandPeriod() const;
    void reset(const std::vector<double> & initialCommand);

private:
    void updateSlopes();

    const std::string name;
    const int minSize {0};
    std::list<std::pair<std::vector<double>, ros::SteadyTime>> buffer;
    decltype(buffer)::iterator left;
    decltype(buffer)::iterator right;
    std::vector<double> slopes;
    ros::WallDuration offset;
    bool enabled {false};
};

#endif // __COMMAND_BUFFER_HPP__
