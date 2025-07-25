
#include <dqrobotics/DQ.h>
#include <dqrobotics/utils/DQ_Constants.h>
#include <dqrobotics/interfaces/webots/DQ_WebotsInterface.h>
#include <dqrobotics/interfaces/webots/robots/URXWebotsRobot.h>

#include <signal.h>

void handle_sigint(int sig){
    std::cout<<"SIGINT received"<<std::endl;
}

int main() {
    //std::signal(SIGINT, handle_sigint);
    if(signal(SIGINT, handle_sigint) == SIG_ERR)
    {
        throw std::runtime_error("::Error setting the signal int handler.");
    }
    auto wb = std::make_shared<DQ_WebotsInterface>();

    wb->connect("ur3");
    wb->set_stepping_mode(true);
    wb->trigger_next_simulation_step();

    auto URX = std::make_shared<URXWebotsRobot>(wb, URXWebotsRobot::MODEL::UR3);

    VectorXd target = (VectorXd(6)<<pi/2,0,0,0,0,0).finished();


    for (int i=0;i<300;i++)
    {
        try {
            URX->set_target_configuration(target);
            auto q = URX->get_configuration();
            DQ x = wb->get_object_pose(wb->get_robot_name());
            std::cout<<i<<std::endl;
            wb->trigger_next_simulation_step();
            raise(SIGINT);
            i++;
        } catch (...) {
            std::cout<<"catch"<<std::endl;
        }


    }
    std::cout<<"finishing...!"<<std::endl;
    wb->reset_simulation();
}
