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

#include <dqrobotics/interfaces/webots/DQ_WebotsRobot.h>



DQ_WebotsRobot::DQ_WebotsRobot(const std::shared_ptr<DQ_WebotsInterface> &webots_interface)
    :wb_{webots_interface}
{

}

DQ_WebotsRobot::DQ_WebotsRobot(const std::shared_ptr<DQ_WebotsInterface> &webots_interface,
                               const std::vector<std::string> &joint_motor_names,
                               const std::vector<std::string> &joint_position_sensor_names)
    :wb_{webots_interface},
    joint_motor_names_{joint_motor_names},
    joint_position_sensor_names_{joint_motor_names}
{

}

void DQ_WebotsRobot::set_target_configuration(const VectorXd &target_configuration)
{
    _get_interface_sptr()->set_joint_target_positions(joint_motor_names_, target_configuration);
}

VectorXd DQ_WebotsRobot::get_configuration()
{
    return _get_interface_sptr()->get_joint_positions(joint_position_sensor_names_);
}

void DQ_WebotsRobot::set_joint_motor_names(const std::vector<std::string>& motor_names)
{
    joint_motor_names_ = motor_names;
}

void DQ_WebotsRobot::set_joint_position_sensor_names(const std::vector<std::string> &position_sensor_names)
{
    joint_position_sensor_names_ = position_sensor_names;
}

void DQ_WebotsRobot::set_joint_motor_and_position_sensor_names(const std::vector<std::string> &motor_names,
                                                               const std::string &sensor_suffix)
{
    set_joint_motor_names(motor_names);
    std::vector<std::string> joint_sensors;
    joint_sensors.reserve(motor_names.size());
    for (auto& name : motor_names)
        joint_sensors.emplace_back(name+sensor_suffix);
    set_joint_position_sensor_names(joint_sensors);
}

std::shared_ptr<DQ_WebotsInterface> DQ_WebotsRobot::_get_interface_sptr() const
{
    return wb_;
}

