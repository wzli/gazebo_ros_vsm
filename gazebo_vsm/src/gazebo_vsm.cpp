#include <gazebo_vsm/gazebo_vsm.hpp>

#include <iostream>

namespace gazebo {

void GazeboVsm::Load(int argc, char** argv) {
    for (int i = 0; i < argc; ++i) {
        puts(argv[i]);
    }
    puts("SYSTEM PLUGIN LOADED");

    _world_created_event = gazebo::event::Events::ConnectWorldCreated(
            [this](std::string world_name) { onWorldCreated(std::move(world_name)); });
}

void GazeboVsm::onWorldCreated(std::string world_name) {
    printf("created %s\r\n", world_name.c_str());
}

GZ_REGISTER_SYSTEM_PLUGIN(GazeboVsm)

}  // namespace gazebo
