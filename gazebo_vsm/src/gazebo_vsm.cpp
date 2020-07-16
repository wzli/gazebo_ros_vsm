#include <gazebo_vsm/gazebo_vsm.hpp>
#include <vsm/zmq_transport.hpp>
#include <yaml-cpp/yaml.h>

#include <exception>
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
    if(_world) {
        throw std::runtime_error("VSM plugin: multiple worlds not supported.");
    }
    _world = gazebo::physics::get_world(world_name);
    if (!_world) {
        throw std::runtime_error("VSM plugin: physics::get_world() fail to return world");
    }
    vsm::MeshNode::Config mesh_config{
            vsm::msecs(1),     // peer update interval
            vsm::msecs(1000),  // entity expiry interval
            8000,         // entity updates size
            {},           // ego sphere
            {
                    "node1",                  // name
                    "udp://127.0.0.1:11611",  // address
                    {0, 0},                   // coordinates
                    0,                        // power radius
                    1,                        // connection_degree
                    20,                       // lookup size
                    0,                        // rank decay
            },
            std::make_shared<vsm::ZmqTransport>("udp://*:11611"),  // transport
            std::make_shared<vsm::Logger>(),                       // logger
    };
}

GZ_REGISTER_SYSTEM_PLUGIN(GazeboVsm)

}  // namespace gazebo
