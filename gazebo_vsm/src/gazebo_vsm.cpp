#include <gazebo_vsm/gazebo_vsm.hpp>
#include <vsm/zmq_transport.hpp>

#include <cstdlib>
#include <exception>
#include <iostream>

namespace gazebo {

void GazeboVsm::Load(int argc, char** argv) {
    // parse arguments for config yaml file path
    const char* yaml_path = nullptr;
    for (int i = 1; i < argc; ++i) {
        yaml_path = strstr(argv[i], "--vsm-config=");
        if (yaml_path) {
            // parse config file
            break;
        }
    }
    // load config yaml
    if (!yaml_path) {
        throw std::runtime_error(
                "VSM plugin: failed to parse param --vsm-config=[vsm_config.yaml]");
    }
    _yaml = YAML::LoadFile(yaml_path + 13);
    // register world creation callback
    _world_created_event = gazebo::event::Events::ConnectWorldCreated(
            [this](std::string world_name) { onWorldCreated(std::move(world_name)); });

    // create logger and register log handler
    _logger = std::make_shared<vsm::Logger>();
    _logger->addLogHandler(
            static_cast<vsm::Logger::Level>(std::stoi(yamlField(_yaml, "verbosity"))),
            [](vsm::msecs time, vsm::Logger::Level level, vsm::Error error, const void* /*data*/,
                    size_t /*len*/) {
                std::cout << "VSM plugin: t " << time.count() << ", lv " << level << ", type "
                          << error.type << ", code " << error.code << ", msg " << error.what()
                          << std::endl;
            });
    // create mesh node
    initMeshNode();
}

void GazeboVsm::initMeshNode() {
    _mesh_node = std::make_unique<vsm::MeshNode>(vsm::MeshNode::Config{
            vsm::msecs(std::stoul(yamlField(_yaml, "peer_update_interval"))),
            vsm::msecs(std::stoul(yamlField(_yaml, "entity_expiry_interval"))),
            std::stoul(yamlField(_yaml, "entity_updates_size")),
            // ego sphere
            {
                    nullptr,  // entity_update_handler
                    std::stoul(yamlField(_yaml, "timestamp_lookup_size")),
            },
            // peer tracker
            {
                    yamlField(_yaml, "name", false),
                    "udp://" + yamlField(_yaml, "address") + ":" + yamlField(_yaml, "port"),
                    _yaml["initial_coordinates"]
                            ? _yaml["initial_coordinates"].as<std::vector<float>>()
                            : std::vector<float>(3),
                    std::stof(yamlField(_yaml, "power_radius")),
                    std::stoul(yamlField(_yaml, "connection_degree")),
                    std::stoul(yamlField(_yaml, "peer_lookup_size")),
                    std::stof(yamlField(_yaml, "peer_rank_decay")),
            },
            std::make_shared<vsm::ZmqTransport>("udp://*:" + yamlField(_yaml, "port")), _logger,
            // std::function<msecs(void)> local_clock
    });
    // spawn worker thread to drive the vsm mesh node
    std::thread mesh_thread([this]() {
        for (;;) {
            _mesh_node->getTransport().poll(vsm::msecs(-1));
        }
    });
    mesh_thread.detach();
}

void GazeboVsm::onWorldCreated(std::string world_name) {
    if (_world) {
        throw std::runtime_error("VSM plugin: multiple worlds not supported.");
    }
    _world = gazebo::physics::get_world(world_name);
    if (!_world) {
        throw std::runtime_error(
                "VSM plugin: physics::get_world() fail to return world " + world_name);
    }
    _logger->log(vsm::Logger::INFO, vsm::Error(STRERR(WORLD_CREATED)));
}

std::string GazeboVsm::yamlField(YAML::Node node, std::string field, bool required) const {
    if (!node) {
        throw std::runtime_error("VSM plugin: yaml node is null.");
    }
    auto scalar = node[field];
    if (!scalar || scalar.Scalar().empty()) {
        if (required) {
            throw std::runtime_error("VSM plugin: missig required config " + field);
        }
        return std::string();
    }
    auto str = scalar.as<std::string>();
    if (str.front() == '$') {
        const char* env = std::getenv(str.c_str() + 1);
        if (required && (!env || !*env)) {
            throw std::runtime_error("VSM plugin: missig required enviornment variable " + str);
        }
        str = env ? env : "";
    }
    return str;
}

GZ_REGISTER_SYSTEM_PLUGIN(GazeboVsm)

}  // namespace gazebo
