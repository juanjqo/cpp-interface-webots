
#include <dqrobotics/DQ.h>
#include <dqrobotics/utils/DQ_Constants.h>
#include <dqrobotics/interfaces/webots/DQ_WebotsInterface.h>
#include <dqrobotics/interfaces/webots/robots/URXWebotsRobot.h>

/*********************************************
 * SIGNAL HANDLER
 * *******************************************/
#include<signal.h>
static std::atomic_bool kill_this_process{false};
void sig_int_handler(int)
{
    kill_this_process = true;
}

int main() {
    // Since Webots captures the SIGINT signals, I initialize Webots first in order to
    // override the SIGINT interruption.
    auto wb = std::make_shared<DQ_WebotsInterface>();
    if(signal(SIGINT, sig_int_handler) == SIG_ERR)
        throw std::runtime_error("::Error setting the signal int handler.");



    wb->connect("ur3");
    wb->set_stepping_mode(true);
    wb->trigger_next_simulation_step();

    auto URX = std::make_shared<URXWebotsRobot>(wb, URXWebotsRobot::MODEL::UR3);

    VectorXd target = (VectorXd(6)<<0,0,0,0,0,0).finished();
    double T = 0.01;
    unsigned int i = 0;
    while(!kill_this_process)
    {
        try {
            target(0) = sin(2*pi*i*T);
            URX->set_target_configuration(target);
            auto q = URX->get_configuration();
            DQ x = wb->get_object_pose(wb->get_robot_name());
            URX->set_target_configuration(target);
            wb->trigger_next_simulation_step();
            i++;
        } catch (const std::runtime_error& e) {
            std::cout<<e.what()<<std::endl;
        }
    }
    std::cout<<"Example finished!"<<std::endl;
    wb->reset_simulation();
}
