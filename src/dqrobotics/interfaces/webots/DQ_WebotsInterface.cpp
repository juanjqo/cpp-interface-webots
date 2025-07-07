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
#include <unordered_map>

class DQ_WebotsInterface::Impl
{
public:
    std::shared_ptr<webots::Supervisor> supervisor_;
    webots::Node* robot_node_;

    std::unordered_map<std::string, webots::PositionSensor*> position_sensor_map_;


    /**
     * @brief _update_position_sensor_map updates the position_sensor_map map if the joint_sensor_name is not already there.
     *                              If the position_sensor_map map is update, this method enable the sensor.
     * @param joint_sensor_name
     * @param position_sensor
     * @param sampling_period
     */
    void _update_position_sensor_map(const std::string& joint_sensor_name,
                                     webots::PositionSensor* position_sensor,
                                     const int& sampling_period = 32)
    {
        auto rtn = position_sensor_map_.try_emplace(joint_sensor_name, position_sensor);
        if (std::get<1>(rtn)) // If the map is update, enable the position sensor.
            position_sensor->enable(sampling_period);
    }


    /**
     * @brief _get_position_sensor gets the PositionSensor pointer given the name of the sensor. The pointer is added to the position_sensor_map
     * @param joint_sensor_name
     * @param sampling_period
     * @return
     */
    webots::PositionSensor* _get_position_sensor(const std::string& joint_sensor_name, const int& sampling_period = 32)
    {
        webots::PositionSensor *sensor = supervisor_->getPositionSensor(joint_sensor_name);
        std::string msg1 = "Joint sensor \""+joint_sensor_name+"\" not found in the current world file. \n";
        _check_pointer(sensor, msg1);
        _update_position_sensor_map(joint_sensor_name, sensor, sampling_period);
        return sensor;
    }

    webots::PositionSensor* _get_position_sensor_from_map(const std::string& joint_sensor_name, const int& sampling_period = 32)
    {
        auto search = position_sensor_map_.find(joint_sensor_name);
        // returns a tuple <bool, int>
        //If the handle is found in the map, returns <true, handle>.

        if (search != position_sensor_map_.end())
        { // handle found in map
            return search->second;
        }
        else
        {   // handle not found in map. Therefore, it is taken from Webots and the map
            // is updated;
            return _get_position_sensor(joint_sensor_name, sampling_period);
        }
    }

    template <typename T>
    void _check_pointer(T pointer, const std::string& msg)
    {
        if (not pointer)
            throw std::runtime_error(msg);
    }

};

DQ_WebotsInterface::DQ_WebotsInterface()
    :robot_node_is_defined_{false}, sampling_period_{32}
{
    impl_ = std::make_shared<DQ_WebotsInterface::Impl>();
    impl_->supervisor_ = std::make_shared<webots::Supervisor>();

}

DQ_WebotsInterface::~DQ_WebotsInterface()
{

}


bool DQ_WebotsInterface::connect(const std::string &host, const int &port, const int &TIMEOUT_IN_MILISECONDS)
{
    throw std::runtime_error("Unsupported!");
}

void DQ_WebotsInterface::trigger_next_simulation_step() const
{
    impl_->_check_pointer(impl_->supervisor_, "Bad call in DQ_WebotsInterface::trigger_next_simulation_step(): You must connect first!");
    impl_->supervisor_->step(32);
}

void DQ_WebotsInterface::set_stepping_mode(const bool &flag) const
{
    throw std::runtime_error("Unsupported!");
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
    for (int i=0;i<jointnames.size();i++)
       joint_positions[i] =  impl_->_get_position_sensor_from_map(jointnames.at(i), sampling_period_)->getValue();
    return joint_positions;
}

void DQ_WebotsInterface::set_joint_positions(const std::vector<std::string> &jointnames, const VectorXd &joint_positions)
{
    throw std::runtime_error("Unsupported!");
}

void DQ_WebotsInterface::set_joint_target_positions(const std::vector<std::string> &jointnames, const VectorXd &joint_target_positions)
{
    throw std::runtime_error("Unsupported!");
}

VectorXd DQ_WebotsInterface::get_joint_velocities(const std::vector<std::string> &jointnames)
{
    throw std::runtime_error("Unsupported!");
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
        impl_->_check_pointer(impl_->robot_node_,"Error in DQ_WebotsInterface::connect(). Possible causes: \n"+msg1+msg2+msg3+msg4);

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
