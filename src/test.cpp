
#include <dqrobotics/DQ.h>
#include <dqrobotics/interfaces/webots/DQ_WebotsInterface.h>

int main() {
    DQ_WebotsInterface wb{};
    wb.connect("ur5");

    for(int i=0;i<100;i++)
    {
        wb.trigger_next_simulation_step();
        auto joint_positions = wb.get_joint_positions({"shoulder_pan_joint_sensor"});
        std::cout<<joint_positions<<std::endl;
    }
}
