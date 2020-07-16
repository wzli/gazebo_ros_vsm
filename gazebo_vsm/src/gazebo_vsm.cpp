#include <gazebo_vsm/gazebo_vsm.hpp>
#include <vsm/zmq_transport.hpp>

#include <cstdlib>
#include <exception>
#include <iostream>

namespace gazebo {

void GazeboVsm::Load(int argc, char** argv) {
    // parse arguments for config file path
    for (int i = 1; i < argc; ++i) {
        const char* yaml_path = strstr(argv[i], "--vsm-config=");
        if(yaml_path) {
            // parse config file
            _yaml = YAML::LoadFile(yaml_path + 13);
            break;
        }
    }
    if(!_yaml) {
        throw std::runtime_error("VSM plugin: failed to parse param --vsm-config=[vsm_config.yaml]");
    }

    puts(yamlString(_yaml["node_name"]).c_str());

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

std::string GazeboVsm::yamlString(const YAML::Node& node) const {
    auto str = node.as<std::string>();
    return str.front() == '$' ? std::getenv(str.c_str() + 1) : str;
};

GZ_REGISTER_SYSTEM_PLUGIN(GazeboVsm)

}  // namespace gazebo
