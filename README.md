![Static Badge](https://img.shields.io/badge/status-experimental-critical)![GitHub License](https://img.shields.io/github/license/juanjqo/cpp-interface-webots)![Static Badge](https://img.shields.io/badge/Written_in-C%2B%2B17-blue)![Static Badge](https://img.shields.io/badge/Webots-R2025a-orange)

# cpp-interface-webots (under development)

<img src=https://github.com/juanjqo/capybara_toolkit/assets/23158313/2e0dbd2d-9b12-4930-9ffe-511d8270de03 width='250'>


|  GH Actions  | SO | Status (C++17) | 
| ------------- | ------------- |------------- |
| | macOS ![Static Badge](https://img.shields.io/badge/Apple_silicon-magenta) | ![Static Badge](https://img.shields.io/badge/beta-yellow)|
| [![CPP Build](https://github.com/juanjqo/cpp-interface-webots/actions/workflows/ubuntu.yaml/badge.svg)](https://github.com/juanjqo/cpp-interface-webots/actions/workflows/ubuntu.yaml)   | Ubuntu LTS ![Static Badge](https://img.shields.io/badge/x64-blue) ![Static Badge](https://img.shields.io/badge/arm64-blue)  |  ![Static Badge](https://img.shields.io/badge/beta-yellow)|
|   | Windows 11 ![Static Badge](https://img.shields.io/badge/x64-blue) ![Static Badge](https://img.shields.io/badge/arm64-blue)   |  ![Static Badge](https://img.shields.io/badge/unsupported-gray) | 

### Prerequisites

1. [DQ Robotics](https://github.com/dqrobotics).
2. [Webots](https://cyberbotics.com/#webots).


### Install (UNIX)

```shell
cd ~/Downloads/ && git clone https://github.com/juanjqo/cpp-interface-webots
cd ~/Downloads/cpp-interface-webots
mkdir build && cd build
cmake ..
make
sudo make install
```

### Example

1. Open the [ur3.wbt](https://github.com/juanjqo/cpp-interface-webots/blob/main/examples/ur3/webots_scene/worlds/ur3.wbt) scene.
2. Run the following code:


#### source file

```cpp
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

```

