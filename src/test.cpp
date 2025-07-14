
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
    wb.set_stepping_mode(true);

    std::vector<std::string> joint_sensors;
    for (auto& name : jointnames)
        joint_sensors.emplace_back(name+"_sensor");

    VectorXd target = (VectorXd(6)<<-pi/2,-pi/2,0,0,0,0).finished();

    for(int i=0;i<100;i++)
    {
        wb.trigger_next_simulation_step();
        wb.set_joint_target_positions(jointnames, target);
        std::cout<<wb.get_joint_positions(joint_sensors).transpose()<<std::endl;
        std::cout<<wb.get_object_pose("duck").translation()<<std::endl;
        std::cout<<wb.get_object_pose("duck").rotation()<<std::endl;

    }
    wb.reset_simulation();
}
