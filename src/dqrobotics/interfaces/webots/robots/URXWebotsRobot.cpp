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

#include <dqrobotics/interfaces/webots/robots/URXWebotsRobot.h>

std::vector<std::string> URXWebotsRobot::_get_jointames_from_model() const
{
    switch (model_) {
    case MODEL::UR3:
        return jointnames_ur5_;
    case MODEL::UR5:
        return jointnames_ur5_;
    case MODEL::UR10:
        throw std::runtime_error("The model UR10 is not available.");
    default:
        throw std::runtime_error("Invalid argument.");
    }
}

URXWebotsRobot::URXWebotsRobot(const std::shared_ptr<DQ_WebotsInterface>& webots_interface,
                               const MODEL& model)
    :DQ_WebotsRobot{webots_interface}, model_{model}
{
    set_joint_motor_and_position_sensor_names(_get_jointames_from_model());
    //base_frame_ = _get_interface_sptr()->get_object_pose(_get_interface_sptr()->get_robot_name());
}

URXWebotsRobot::URXWebotsRobot(const std::shared_ptr<DQ_WebotsInterface> &webots_interface,
                               const std::vector<std::string> &joint_motor_names,
                               const std::vector<std::string> &joint_position_sensor_names,
                               const MODEL &model)
    :DQ_WebotsRobot{webots_interface}
{
    set_joint_motor_names(joint_position_sensor_names);
    set_joint_position_sensor_names(joint_position_sensor_names);
}

/*
DQ_SerialManipulatorDH URXWebotsRobot::kinematics()
{
    auto kin = DQ_SerialManipulatorDH(_get_dh_matrix());
    kin.set_reference_frame(base_frame_);
    kin.set_base_frame(base_frame_);
    return kin;
}
*/


/**
 * @brief URXWebotsRobot::_get_dh_matrix eturns a matrix related to the D-H parameters of the
 *                        Universal Robot {}, which is
 *                        defined as
 *
 *                        Matrix<Xd raw_dh_matrix;
 *                        raw_franka_mdh << theta,
 *                                              d,
 *                                              a,
 *                                           alpha,
 *                                            type_of_joints;
 * Source: https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/
 * @param model  ex: UR3,UR5, UR10
 *
 * @return MatrixXd raw_dh_matrix a matrix related to the D-H parameters
 */
MatrixXd URXWebotsRobot::_get_dh_matrix() const
{
    const double pi = M_PI;
    auto R = DQ_JointType::REVOLUTE;
    switch (model_){
        case MODEL::UR3:
        {
            Matrix<double,5,6> raw_dh_matrix(5,6);
            raw_dh_matrix << 0, 0, 0, 0, 0, 0,
                             0.15185, 0, 0, 0.13105, 0.08535, 0.0921,
                             0, -0.24355, -0.2132, 0, 0, 0,
                             pi/2, 0, 0, pi/2, -pi/2, 0,
                             R, R, R, R, R, R;
            return raw_dh_matrix;
        }
        case MODEL::UR5:
        {
            throw std::runtime_error("The model UR5 is not available.");
            Matrix<double,5,6> raw_dh_matrix(5,6);

            raw_dh_matrix <<  -pi/2, -pi/2, 0, -pi/2, 0, 0,
                0.089159-0.02315, 0, 0, 0.10915, 0.09465, 0.0823,
                0, -0.425, -0.39225, 0, 0, 0,
                pi/2,0,0,pi/2,-pi/2,0,
                R,      R,       R,         R,         R,      R;

            return raw_dh_matrix;
        }
        case MODEL::UR10:
        {
            throw std::runtime_error("The model UR10 is not available.");

        }
        default:
        {
            throw std::runtime_error("Invalid argument.");
        }
    }
}
