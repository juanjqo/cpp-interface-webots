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
#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterface.h>
#include <atomic>
#include <memory>

using namespace DQ_robotics;
using namespace Eigen;


class DQ_WebotsInterface: public DQ_CoppeliaSimInterface
{
public:
    DQ_WebotsInterface();
    ~DQ_WebotsInterface();

    //-----Concrete methods from DQ_CoppeliaSimInterface---------------------------------------------
    bool connect(const std::string& host, const int& port, const int&TIMEOUT_IN_MILISECONDS) override;
    void trigger_next_simulation_step() const override;
    void set_stepping_mode(const bool& flag) const override;
    void start_simulation() const override;
    void stop_simulation()  const override;

    int get_object_handle(const std::string& objectname) override;
    std::vector<int> get_object_handles(const std::vector<std::string>& objectnames) override;

    DQ   get_object_translation(const std::string& objectname) override;
    void set_object_translation(const std::string& objectname, const DQ& t) override;

    DQ   get_object_rotation   (const std::string& objectname) override;
    void set_object_rotation   (const std::string& objectname, const DQ& r) override;

    DQ   get_object_pose       (const std::string& objectname) override;
    void set_object_pose       (const std::string& objectname, const DQ& h) override;

    VectorXd get_joint_positions(const std::vector<std::string>& jointnames) override;
    void     set_joint_positions(const std::vector<std::string>& jointnames,
                                 const VectorXd& joint_positions) override;
    void     set_joint_target_positions(const std::vector<std::string>& jointnames,
                                        const VectorXd& joint_target_positions) override;

    VectorXd get_joint_velocities(const std::vector<std::string>& jointnames) override;
    void     set_joint_target_velocities(const std::vector<std::string>& jointnames,
                                         const VectorXd& joint_target_velocities) override;

    void     set_joint_target_forces(const std::vector<std::string>& jointnames,
                                     const VectorXd& forces) override;
    VectorXd get_joint_forces(const std::vector<std::string>& jointnames) override;
    //----------------------------------------------------------------------------------------------------


    bool connect(const std::string& robot_definition);
    void set_sampling_period(const int& sampling_period);

protected:
    std::string robot_definition_;
    std::atomic<bool> robot_node_is_defined_;

private:
    class Impl;
    std::shared_ptr<Impl> impl_;
};
