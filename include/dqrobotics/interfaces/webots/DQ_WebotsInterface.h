/**
(C) Copyright 2011-2025 DQ Robotics Developers

This file is based on DQ Robotics.

    DQ Robotics is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    DQ Robotics is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.

DQ Robotics website: dqrobotics.github.io

Contributors:
- Juan Jose Quiroz Omana
       - Responsible for the original implementation.

*/

#pragma once

#pragma once
#include <dqrobotics/DQ.h>
#include <atomic>
#include <memory>

using namespace DQ_robotics;
using namespace Eigen;


class DQ_WebotsInterface
{
public:
    DQ_WebotsInterface(const int& sampling_period = 32);
    virtual ~DQ_WebotsInterface() = default;


    bool connect(const std::string& robot_definition);

    void set_sampling_period(const int& sampling_period);
    int  get_sampling_period() const;

    void reset_simulation() const;
    void set_stepping_mode(const bool& flag) const;
    void trigger_next_simulation_step() const;

    VectorXd get_joint_positions(const std::vector<std::string>& jointnames);
    void     set_joint_target_positions(const std::vector<std::string>& jointnames,
                                    const VectorXd& joint_target_positions);

    VectorXd get_joint_velocities(const std::vector<std::string>& jointnames);

private:
    std::string robot_definition_;
    std::atomic<bool> robot_node_is_defined_;

    class Impl;
    std::shared_ptr<Impl> impl_;
    void _check_connection(const std::string& msg) const;
    std::string error_msg_layout_ = "Bad call in DQ_WebotsInterface::";

};
