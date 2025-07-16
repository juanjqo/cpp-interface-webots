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
    std::unordered_map<std::string, webots::Node*> nodes_map_;

    /**
     * @brief _get_position_sensor gets the PositionSensor pointer given the name of the sensor.
     *                       The pointer is added to the position_sensor_map and the sensor is enabled.
     * @param sensor_name The name of the position sensor.
     * @return The desired PositionSensor pointer.v
     */
    webots::PositionSensor* _get_position_sensor(const std::string& sensor_name)
    {
        webots::PositionSensor *sensor = supervisor_->getPositionSensor(sensor_name);
        check_pointer(sensor, "Joint sensor \""+sensor_name+"\" not found in the current world file. \n");

        // Update the position sensor map
        auto [ith, found] = position_sensor_map_.try_emplace(sensor_name, sensor);
        if (found) // If the map is updated, enable the position sensor.
            sensor->enable(sampling_period_);
        return sensor;
    }

    /**
     * @brief _get_joint_motor gets the Motor pointer given the name of the motor. The pointer is added
     *                      to the motor_sensor_map_.
     * @param motor_name The name of the motor
     * @return The desired Motor pointer.
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
     * @brief _get_node gets the Node pointer given the name of the object. The pointer is added
     *                      to the nodes_map_.
     * @param motor_name The name of the object.
     * @return The desired Node pointer.
     */
    webots::Node* _get_node(const std::string& objectname)
    {
        webots::Node* node = supervisor_->getFromDef(objectname);
        check_pointer(node, "Object \""+objectname+"\" not found in the current world file. \n");
        nodes_map_.try_emplace(objectname, node);
        return node;
    }

    /**
     * @brief _get_element_from_map gets the element from the specified map. If the element is not found,
     *                  This method calls the specified function.
     * @param name The name of the element.
     * @param map The map used to store the specific type of elements.
     * @param f The function to be call if the element is not found in the map
     * @return The desired T-type pointer
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
    int sampling_period_;


    Impl(const int& sampling_period)
        :sampling_period_{sampling_period}
    {

    };

    Impl() = delete;


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
     * @brief get_object_node_from_map
     * @param objectname
     * @return
     */
    webots::Node* get_object_node_from_map(const std::string& objectname)
    {
        return _get_element_from_map<webots::Node*>(objectname, nodes_map_,
                                                     std::bind(&DQ_WebotsInterface::Impl::_get_node, this, std::placeholders::_1));
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
    void check_pointer(T pointer, const std::string& msg) const
    {
        if (not pointer)
            throw std::runtime_error(msg);
    }


    /**
     * @brief convert_to_std_vector
     * @param raw_data
     * @return
     */
    template <std::size_t N>
    std::array<double, N> convert_to_std_vector(const double* raw_data) const
    {
        std::array<double, N> vec;
        std::copy(raw_data, raw_data + N, vec.begin());
        return vec;
    }


};


/**
 * @brief DQ_WebotsInterface::DQ_WebotsInterface constructor of the class.
 * @param sampling_period The sampling period (default 32). This parameter corresponds to the duration of the control steps,
 *                     i.e., the wb_robot_step function shall compute 32 milliseconds of simulation and then return.
 *                     This duration specifies an amount of simulated time, not real (wall clock) time,
 *                     so it may actually take 1 millisecond or one minute of real time, depending on
 *                     the complexity of the simulated world.
 */
DQ_WebotsInterface::DQ_WebotsInterface(const int &sampling_period)
{
    impl_ = std::make_shared<DQ_WebotsInterface::Impl>(sampling_period);
    impl_->supervisor_ = std::make_shared<webots::Supervisor>();
}


/**
 * @brief DQ_WebotsInterface::trigger_next_simulation_step performs a simulation step.
 */
void DQ_WebotsInterface::trigger_next_simulation_step() const
{
    _check_connection(error_msg_layout_+std::string(__func__));
    impl_->supervisor_->step(get_sampling_period());
}


/**
 * @brief DQ_WebotsInterface::get_object_pose returns a unit dual quaternion that represents
 *        the object pose in the Webots scene with respect to the absolute frame.
 * @param objectname The name of the object in the Webots scene.
 * @return the desired object pose
 */
DQ DQ_WebotsInterface::get_object_pose(const std::string &objectname)
{
    const DQ p = _get_object_translation(objectname);
    const DQ r = _get_object_rotation(objectname);
    return r+0.5*E_*p*r;
}


/**
 * @brief DQ_WebotsInterface::set_object_pose sets the pose of an object in the Webots scene.
 * @param objectname The name of the object in the Webots scene.
 * @param Pose A unit dual qualternion that represents the desired object pose with respect to the absolute frame.
 */
void DQ_WebotsInterface::set_object_pose(const std::string &objectname, const DQ &pose)
{
    if (!is_unit(pose))
        throw std::runtime_error(error_msg_layout_+std::string(__func__)+". The pose must be a unit dual quaternion!");
    const DQ t = pose.translation();
    const DQ r = pose.rotation();
    _set_object_rotation(objectname, r);
    _set_object_translation(objectname, t);
}


/**
 * @brief DQ_WebotsInterface::_set_object_translation sets the translation of an object
 *        in the Webots scene.
 * @param objectname the name of the object
 * @param t The pure quaternion that represents the desired position with respect to the absolute frame.
 */
void DQ_WebotsInterface::_set_object_translation(const std::string &objectname, const DQ &t)
{
    webots::Field* translation_field= impl_->get_object_node_from_map(objectname)->getField("translation");
    VectorXd vec_t = t.vec3();
    const double values[3] = {vec_t(0),vec_t(1),vec_t(2)};
    translation_field->setSFVec3f(values);

}


/**
 * @brief DQ_WebotsInterface::_set_object_rotation sets the rotation of an object in the Webots scene.
 * @param objectname the name of the object
 * @param r A unit quaternion that represents the desired rotation with respect to the absolute frame.
 */
void DQ_WebotsInterface::_set_object_rotation(const std::string &objectname, const DQ &r)
{
    webots::Field *rotation_field = impl_->get_object_node_from_map(objectname)->getField("rotation");
    const VectorXd vec_r = r.vec4();
    const VectorXd axis = r.rotation_axis().vec3();
    const double angle = r.rotation_angle();
    const double x = axis[0];
    const double y = axis[1];
    const double z = axis[2];
    const double values[4] = {x,y,z, angle};
    rotation_field->setSFRotation(values);
}


/**
 * @brief DQ_WebotsInterface::_get_object_translation returns a pure quaternion that represents the position
 *        of an object in the Webots scene with respect to the absolute frame.
 * @param objectname The name of the object.
 * @return The translation of the object.
 */
DQ DQ_WebotsInterface::_get_object_translation(const std::string &objectname)
{
    webots::Field* translation_field = impl_->get_object_node_from_map(objectname)->getField("translation");
    const double* t = translation_field->getSFVec3f();
    auto t_vec = impl_->convert_to_std_vector<POS_SIZE_>(t);
    const auto& [x,y,z] = t_vec; //Aliasing
    return  x*i_ + y*j_ + z*k_;
}


/**
 * @brief DQ_WebotsInterface::_get_object_rotation returns a unit quaternion that represents the rotation
 *        of an object in the Webots scene with respect to the absolute frame.
 * @param objectname the name of the object.
 * @return The object rotation
 */
DQ DQ_WebotsInterface::_get_object_rotation(const std::string &objectname)
{
    webots::Field *rotation_field = impl_->get_object_node_from_map(objectname)->getField("rotation");
    const double *rv = rotation_field->getSFRotation();

    auto rv_vec = impl_->convert_to_std_vector<ROT_SIZE_>(rv);
    const auto& [x,y,z,angle] = rv_vec; //Aliasing

    DQ n = (x*i_ + y*j_ + z*k_).normalize();
    return cos(angle/2) + n*sin(angle/2);
}


/**
 * @brief DQ_WebotsInterface::set_stepping_mode
 * @param flag
 */
void DQ_WebotsInterface::set_stepping_mode(const bool &flag) const
{
    _check_connection(error_msg_layout_+std::string(__func__));
    auto att = impl_->robot_node_->getField("synchronization");
    if (not att)
        throw std::runtime_error("Not synchronization field found in the robot node!");
    att->setSFBool(flag);
}



/**
 * @brief DQ_WebotsInterface::get_joint_positions gets the joint positions from Webots.
 * @param jointnames The name of the joint position sensors.
 * @return The desired joint positions.
 */
VectorXd DQ_WebotsInterface::get_joint_positions(const std::vector<std::string> &jointnames)
{
    const int n = jointnames.size();
    VectorXd joint_positions(n);
    for (int i=0;i<n;i++)
       joint_positions[i] = impl_->get_position_sensor_from_map(jointnames.at(i))->getValue();
    return joint_positions;
}

/**
 * @brief DQ_WebotsInterface::set_joint_target_positions sets the joint target positions on Webots.
 * @param jointnames The name of the joint motors.
 * @param joint_target_positions The desired joint target positions.
 */
void DQ_WebotsInterface::set_joint_target_positions(const std::vector<std::string> &jointnames, const VectorXd &joint_target_positions)
{
    impl_->check_sizes(jointnames, joint_target_positions,
                       error_msg_layout_+std::string(__func__)+": arguments have different sizes!");
    for (int i=0;i<jointnames.size();i++)
        impl_->get_joint_motor_from_map(jointnames.at(i))->setPosition(joint_target_positions[i]);
}


/**
 * @brief DQ_WebotsInterface::test_proto
 * @param DEF
 */
void DQ_WebotsInterface::test_proto(const std::string &DEF)
{
    ///auto node = impl_->supervisor_->get
}


/**
 * @brief DQ_WebotsInterface::get_robot_name
 * @return
 */
std::string DQ_WebotsInterface::get_robot_name() const
{
    return DEF_;
}


/*
VectorXd DQ_WebotsInterface::get_joint_velocities(const std::vector<std::string> &jointnames)
{
    const int n = jointnames.size();
    VectorXd joint_velocities(n);
    for (int i=0;i<n;i++)
        joint_velocities[i] = impl_->get_joint_motor_from_map(jointnames.at(i))->getVelocity();
    return joint_velocities;
}
*/



/**
 * @brief DQ_WebotsInterface::connect This method connects with a robot node element on Webots. The robot node must have
 *                          a DEF tag defined, a controller attribute set on "<extern>", and a supervisor attribute set on TRUE.
 * @param robot_definition The DEF tag defined on the Webots robot node.
 * @return True if the connection is ok. False otherwise.
 */
bool DQ_WebotsInterface::connect(const std::string &robot_definition)
{
    if (not impl_->robot_node_){

        //impl_->robot_node_ = impl_->supervisor_->getFromDef(robot_definition);
        impl_->robot_node_ = impl_->get_object_node_from_map(robot_definition);

        std::string msg1 = "1. No DEF \""+robot_definition+"\" node found in the current world file. \n";
        std::string msg2 = "2. The robot controller is not set to <extern>. \n";
        std::string msg3 = "3. The supervisor is not set to TRUE. \n";
        std::string msg4 = "Save the scene, and open the world again. \n";
        impl_->check_pointer(impl_->robot_node_,
                             error_msg_layout_+std::string(__func__)+". Possible causes: \n"+msg1+msg2+msg3+msg4);
    }
    DEF_ = robot_definition;
    return impl_->robot_node_;

}


/**
 * @brief DQ_WebotsInterface::set_sampling_period specifies the duration of the control steps. For instance, if you use
 *                     set_sampling_period(32), this function shall compute 32 milliseconds of simulation and then return.
 *                     This duration specifies an amount of simulated time, not real (wall clock) time,
 *                     so it may actually take 1 millisecond or one minute of real time, depending on the complexity of
 *                     the simulated world.
 *
 *                     check more in https://cyberbotics.com/doc/guide/controller-programming
 *
 * @param sampling_period The desired sampling period.
 */
void DQ_WebotsInterface::set_sampling_period(const int &sampling_period)
{
    impl_->sampling_period_ = sampling_period;
}


/**
 * @brief DQ_WebotsInterface::get_sampling_period gets the sampling period. This value correspond to the duration of the control steps.
 * @return The sampling period.
 */
int DQ_WebotsInterface::get_sampling_period() const
{
    return impl_->sampling_period_;
}


/**
 * @brief DQ_WebotsInterface::reset_simulation sends a request to the simulator process, asking it to reset the simulation at the end of the step.
 *
 *   Source: https://cyberbotics.com/doc/reference/supervisor?tab-language=c++#wb_supervisor_simulation_reset
 */
void DQ_WebotsInterface::reset_simulation() const
{
    _check_connection(error_msg_layout_+std::string(__func__));
    impl_->supervisor_->simulationReset();
}


/**
 * @brief DQ_WebotsInterface::_check_connection throws an exception if the robot node is not valid.
 * @param msg The error message that you want to display to the user.
 */
void DQ_WebotsInterface::_check_connection(const std::string &msg) const
{
    if (not impl_->robot_node_)
        throw std::runtime_error(msg+"\n You must connect first. \n");
}


