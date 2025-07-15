
#include <dqrobotics/DQ.h>
#include <dqrobotics/utils/DQ_Constants.h>
#include <dqrobotics/interfaces/webots/DQ_WebotsInterface.h>
#include <dqrobotics/interfaces/webots/robots/URXWebotsRobot.h>


int main() {
    auto wb = std::make_shared<DQ_WebotsInterface>();
    wb->connect("ur3");
    wb->set_stepping_mode(true);
    wb->trigger_next_simulation_step();

    auto URX = std::make_shared<URXWebotsRobot>(wb, URXWebotsRobot::MODEL::UR3);

    VectorXd target = (VectorXd(6)<<pi/2,0,0,0,0,0).finished();

    for(int i=0;i<100;i++)
    {
        URX->set_target_configuration(target);
        auto q = URX->get_configuration();
        wb->trigger_next_simulation_step();
    }
    wb->reset_simulation();
}
