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
#include <dqrobotics/interfaces/webots/DQ_WebotsRobot.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulatorDH.h>

class URXWebotsRobot: public DQ_WebotsRobot
{
public:
    enum class MODEL{UR3, UR5, UR10};
private:
    DQ base_frame_;
    URXWebotsRobot::MODEL model_;
    const std::vector<std::string> jointnames_ur5_ = {"shoulder_pan_joint",
                                                 "shoulder_lift_joint",
                                                 "elbow_joint",
                                                 "wrist_1_joint",
                                                 "wrist_2_joint",
                                                 "wrist_3_joint",
                                                 };
    std::vector<std::string> _get_jointames_from_model() const;
    MatrixXd _get_dh_matrix() const;
public:
    URXWebotsRobot(const std::shared_ptr<DQ_WebotsInterface>& webots_interface,
                   const MODEL& model);
    URXWebotsRobot(const std::shared_ptr<DQ_WebotsInterface>& webots_interface,
                   const std::vector<std::string>& joint_motor_names,
                   const std::vector<std::string>& joint_position_sensor_names,
                   const MODEL& model);
    //DQ_SerialManipulatorDH kinematics();

};

