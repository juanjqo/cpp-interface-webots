
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



    wb->connect("heron");
    wb->set_stepping_mode(true);
    wb->trigger_next_simulation_step();

    std::vector<std::string> motors{"left_motor", "right_motor"};



    VectorXd target = VectorXd::Zero(2);
    double T = 0.01;
    unsigned int i = 0;
    while(!kill_this_process)
    {
        try {
            target(0) = 10;
            wb->set_joint_target_velocities(motors, target);
            auto velocities = wb->get_joint_velocities(motors);
            std::cout<<velocities.transpose()<<std::endl;
            wb->trigger_next_simulation_step();
            i++;
        } catch (const std::runtime_error& e) {
            std::cout<<e.what()<<std::endl;
        }
    }
    std::cout<<"Example finished!"<<std::endl;
    wb->reset_simulation();
}
