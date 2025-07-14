
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

    DQ t = 0.3*i_ + 0.3*j_+0.1*k_;
    double phi = pi/2;
    DQ n = k_;
    DQ r = cos(phi/2) + n*sin(phi/2);
    DQ x = r + 0.5*E_*t*r;

    for(int i=0;i<100;i++)
    {

        wb.set_joint_target_positions(jointnames, target);
        std::cout<<wb.get_joint_positions(joint_sensors).transpose()<<std::endl;
        std::cout<<wb.get_object_pose("duck").translation()<<std::endl;
        std::cout<<wb.get_object_pose("duck").rotation()<<std::endl;
        wb.set_object_pose("duck", x);
        wb.trigger_next_simulation_step();

    }
    wb.reset_simulation();
}
