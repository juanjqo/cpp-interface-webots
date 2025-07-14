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

#include <dqrobotics/interfaces/webots/DQ_WebotsInterface.h>
#include <webots/Supervisor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Motor.hpp>
#include <unordered_map>

class DQ_WebotsInterface::Impl
{
protected:
    std::unordered_map<std::string, webots::PositionSensor*> position_sensor_map_;
    std::unordered_map<std::string, webots::Motor*> motor_sensor_map_;

    /**
     * @brief _get_position_sensor gets the PositionSensor pointer given the name of the sensor.
     *                       The pointer is added to the position_sensor_map and the sensor is enabled.
     * @param sensor_name The name of the position sensor.
     * @return The desired PositionSensor pointer.
     */
    webots::PositionSensor* _get_position_sensor(const std::string& sensor_name)
    {
        webots::PositionSensor *sensor = supervisor_->getPositionSensor(sensor_name);
        check_pointer(sensor, "Joint sensor \""+sensor_name+"\" not found in the current world file. \n");

        // Update the position sensor map
        auto rtn = position_sensor_map_.try_emplace(sensor_name, sensor);
        if (std::get<1>(rtn)) // If the map is updated, enable the position sensor.
            sensor->enable(sampling_period_);
        return sensor;
    }

    /**
     * @brief _get_joint_motor
     * @param motor_name
     * @return
     */
    webots::Motor* _get_joint_motor(const std::string& motor_name)
    {
        webots::Motor* motor = supervisor_->getMotor(motor_name);
        check_pointer(motor, "Joint \""+motor_name+"\" not found in the current world file. \n");
        // Update the motor sensor map
        motor_sensor_map_.try_emplace(motor_name, motor);
        return motor;
    }

    /**
     * @brief _get_element_from_map
     * @param name
     * @param map
     * @param map_type
     * @return
     */
    template <typename T, typename M>
    T _get_element_from_map(const std::string& name, M& map, const std::function<T(const std::string& name)>& f)
    {
        auto search = map.find(name);
        if (search != map.end())
            return search->second;
        else
            return f(name);
    }


public:
    std::shared_ptr<webots::Supervisor> supervisor_;
    webots::Node* robot_node_;
    int sampling_period_{32};

    Impl()
    {

    };


    /**
     * @brief get_joint_motor_from_map
     * @param joint_motor_name
     * @return
     */
    webots::Motor* get_joint_motor_from_map(const std::string& joint_motor_name)
    {
        return _get_element_from_map<webots::Motor*>(joint_motor_name, motor_sensor_map_,
                             std::bind(&DQ_WebotsInterface::Impl::_get_joint_motor, this, std::placeholders::_1));
    }

    /**
     * @brief get_position_sensor_from_map
     * @param joint_sensor_name
     * @return
     */
    webots::PositionSensor* get_position_sensor_from_map(const std::string& joint_sensor_name)
    {
        return _get_element_from_map<webots::PositionSensor*>(joint_sensor_name, position_sensor_map_,
                             std::bind(&DQ_WebotsInterface::Impl::_get_position_sensor, this, std::placeholders::_1));
    }

    /**
     * @brief check_sizes This method throws an exception with a desired message if
                           the sizes of v1 and v2 are different.
     * @param v1 The first vector to be compared (Eigen::VectorXd or std::vector<>)
     * @param v2 The second vector to be compared (Eigen::VectorXd or std::vector<>)
     * @param error_message The message to be displayed when the exception is raised.
     */
    template <typename T, typename U>
    void check_sizes(const T &v1,
                      const U &v2,
                      const std::string& error_message) const
    {
        if (static_cast<std::size_t>(v1.size()) != static_cast<std::size_t>(v2.size()))
            throw std::runtime_error(error_message);
    }

    /**
     * @brief _check_pointer
     * @param pointer
     * @param msg
     */
    template <typename T>
    void check_pointer(T pointer, const std::string& msg)
    {
        if (not pointer)
            throw std::runtime_error(msg);
    }

};

DQ_WebotsInterface::DQ_WebotsInterface()
    :robot_node_is_defined_{false}
{
    impl_ = std::make_shared<DQ_WebotsInterface::Impl>();
    impl_->supervisor_ = std::make_shared<webots::Supervisor>();
    set_sampling_period(32); //Default

}


bool DQ_WebotsInterface::connect(const std::string &host, const int &port, const int &TIMEOUT_IN_MILISECONDS)
{
    throw std::runtime_error("Unsupported!");
}

void DQ_WebotsInterface::trigger_next_simulation_step() const
{
    _check_connection("Bad call in DQ_WebotsInterface::trigger_next_simulation_step(). ");
    impl_->supervisor_->step(32);
}

void DQ_WebotsInterface::set_stepping_mode(const bool &flag) const
{
    _check_connection("Bad call in DQ_WebotsInterface::set_stepping_mode(). ");
    auto att = impl_->robot_node_->getField("synchronization");
    if (not att)
        throw std::runtime_error("Not synchronization field found in the robot node!");
    att->setSFBool(flag);
}

void DQ_WebotsInterface::start_simulation() const
{
    throw std::runtime_error("Unsupported!");
}

void DQ_WebotsInterface::stop_simulation() const
{
    throw std::runtime_error("Unsupported!");
}

int DQ_WebotsInterface::get_object_handle(const std::string &objectname)
{
    throw std::runtime_error("Unsupported!");
}

std::vector<int> DQ_WebotsInterface::get_object_handles(const std::vector<std::string> &objectnames)
{
    throw std::runtime_error("Unsupported!");
}

DQ DQ_WebotsInterface::get_object_translation(const std::string &objectname)
{
    throw std::runtime_error("Unsupported!");
}

void DQ_WebotsInterface::set_object_translation(const std::string &objectname, const DQ &t)
{
    throw std::runtime_error("Unsupported!");
}

DQ DQ_WebotsInterface::get_object_rotation(const std::string &objectname)
{
    throw std::runtime_error("Unsupported!");
}

void DQ_WebotsInterface::set_object_rotation(const std::string &objectname, const DQ &r)
{
    throw std::runtime_error("Unsupported!");
}

DQ DQ_WebotsInterface::get_object_pose(const std::string &objectname)
{
    throw std::runtime_error("Unsupported!");
}

void DQ_WebotsInterface::set_object_pose(const std::string &objectname, const DQ &h)
{
    throw std::runtime_error("Unsupported!");
}

VectorXd DQ_WebotsInterface::get_joint_positions(const std::vector<std::string> &jointnames)
{
    const int n = jointnames.size();
    VectorXd joint_positions(n);
    for (int i=0;i<n;i++)
       joint_positions[i] = impl_->get_position_sensor_from_map(jointnames.at(i))->getValue();
    return joint_positions;
}

void DQ_WebotsInterface::set_joint_positions(const std::vector<std::string> &jointnames, const VectorXd &joint_positions)
{
    throw std::runtime_error("Unsupported!");
}

void DQ_WebotsInterface::set_joint_target_positions(const std::vector<std::string> &jointnames, const VectorXd &joint_target_positions)
{
    impl_->check_sizes(jointnames, joint_target_positions,
                       "Bad call in DQ_WebotsInterface::set_joint_target_positions. jointnames and joint_target_positions have different sizes!");
    for (int i=0;i<jointnames.size();i++)
        impl_->get_joint_motor_from_map(jointnames.at(i))->setPosition(joint_target_positions[i]);
}

VectorXd DQ_WebotsInterface::get_joint_velocities(const std::vector<std::string> &jointnames)
{
    const int n = jointnames.size();
    VectorXd joint_velocities(n);
    for (int i=0;i<n;i++)
        joint_velocities[i] = impl_->get_joint_motor_from_map(jointnames.at(i))->getVelocity();
    return joint_velocities;
}

void DQ_WebotsInterface::set_joint_target_velocities(const std::vector<std::string> &jointnames, const VectorXd &joint_target_velocities)
{
    throw std::runtime_error("Unsupported!");
}

void DQ_WebotsInterface::set_joint_target_forces(const std::vector<std::string> &jointnames, const VectorXd &forces)
{
    throw std::runtime_error("Unsupported!");
}

VectorXd DQ_WebotsInterface::get_joint_forces(const std::vector<std::string> &jointnames)
{
    throw std::runtime_error("Unsupported!");
}



/**
 * @brief DQ_WebotsInterface::connect
 * @param robot_definition
 * @return
 */
bool DQ_WebotsInterface::connect(const std::string &robot_definition)
{

    robot_definition_ = robot_definition;
    if (not robot_node_is_defined_){

        impl_->robot_node_ = impl_->supervisor_->getFromDef(robot_definition_);


        std::string msg1 = "1. No DEF \""+robot_definition_+"\" node found in the current world file. \n";
        std::string msg2 = "2. The robot controller is not set to <extern>. \n";
        std::string msg3 = "3. The supervisor is not set to TRUE. \n";
        std::string msg4 = "Save the scene, and open the world again. \n";
        impl_->check_pointer(impl_->robot_node_,"Error in DQ_WebotsInterface::connect(). Possible causes: \n"+msg1+msg2+msg3+msg4);

        if (impl_->robot_node_)
        {
            robot_node_is_defined_ = true;
            return true;
        }
        else
        {
            robot_node_is_defined_ = false;
            return false;
        }
    }else{
        return true;
    }

}

void DQ_WebotsInterface::set_sampling_period(const int &sampling_period)
{
    impl_->sampling_period_ = sampling_period;
}

void DQ_WebotsInterface::reset_simulation() const
{
    _check_connection("Bad call in DQ_WebotsInterface::reset_simulation(). ");
    impl_->supervisor_->simulationReset();
}

void DQ_WebotsInterface::_check_connection(const std::string &msg) const
{
    if (not robot_node_is_defined_)
        throw std::runtime_error(msg+"\n You must connect first. \n");
}


