// Trajectory.hpp
#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <string>

gtsam::Matrix loadTrajectoryData(const std::string &filepath);
gtsam::Vector getTrajectoryPositionAtIndex(const gtsam::Matrix &trajectory_data,
                                           int index);

#endif // TRAJECTORY_HPP
