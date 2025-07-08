
#include <dqrobotics/DQ.h>
#include <dqrobotics/interfaces/webots/DQ_WebotsInterface.h>
#include <dqrobotics/utils/DQ_Constants.h>

std::vector<std::string> jointnames = {"shoulder_pan_joint",
                                       "shoulder_lift_joint",
                                       "elbow_joint",
                                       "wrist_1_joint",
                                       "wrist_2_joint",
                                       "wrist_3_joint",
                                       };
int main() {
    DQ_WebotsInterface wb{};
    wb.connect("ur5");

    std::vector<std::string> joint_sensors;
    for (auto& name : jointnames)
        joint_sensors.push_back(name+"_sensor");

    VectorXd target = (VectorXd(6)<<-pi/2,0,0,0,0,0).finished();

    for(int i=0;i<100;i++)
    {
        wb.trigger_next_simulation_step();
        auto joint_positions = wb.get_joint_positions(joint_sensors);
        wb.set_joint_target_positions(jointnames, target);
        std::cout<<joint_positions.transpose()<<std::endl;
    }
}
