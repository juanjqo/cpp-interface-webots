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
#include <dqrobotics/DQ.h>
#include <dqrobotics/interfaces/webots/DQ_WebotsInterface.h>
#include <memory>

using namespace DQ_robotics;
using namespace Eigen;

class DQ_WebotsRobot
{
private:
    std::vector<std::string> joint_motor_names_;
    std::vector<std::string> joint_position_sensor_names_;
    std::shared_ptr<DQ_WebotsInterface> wb_;
protected:
    std::shared_ptr<DQ_WebotsInterface> _get_interface_sptr() const;


public:
    DQ_WebotsRobot(const std::shared_ptr<DQ_WebotsInterface>& webots_interface);
    DQ_WebotsRobot(const std::shared_ptr<DQ_WebotsInterface>& webots_interface,
                   const std::vector<std::string>& joint_motor_names,
                   const std::vector<std::string>& joint_position_sensor_names);
    virtual ~DQ_WebotsRobot() = default;

    void set_target_configuration(const VectorXd& target_configuration);
    VectorXd get_configuration();


    void set_joint_motor_names(const std::vector<std::string>& motor_names);
    void set_joint_position_sensor_names(const std::vector<std::string>& position_sensor_names);
    void set_joint_motor_and_position_sensor_names(const std::vector<std::string>& motor_names,
                                                    const std::string& sensor_suffix="_sensor");


};

