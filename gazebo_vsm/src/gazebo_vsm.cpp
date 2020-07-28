#include <gazebo_vsm/gazebo_vsm.hpp>
#include <vsm/zmq_transport.hpp>

#include <cstdlib>
#include <exception>
#include <regex>
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
    // assert synced_entities are defined
    auto synced_entities = _yaml["synced_entities"];
    if (!synced_entities.IsDefined() || synced_entities.IsNull() || !synced_entities.IsSequence()) {
        throw std::runtime_error("VSM plugin: missing required config synced_entities: []");
    }
    // register gazebo callbacks
    _world_created_event = gazebo::event::Events::ConnectWorldCreated(
            [this](std::string world_name) { onWorldCreated(std::move(world_name)); });

    _world_update_begin_event = gazebo::event::Events::ConnectWorldUpdateBegin(
            [this](const common::UpdateInfo& update_info) { onWorldUpdateBegin(update_info); });

    _world_update_end_event =
            gazebo::event::Events::ConnectWorldUpdateEnd([this]() { onWorldUpdateEnd(); });

    _add_entity_event = gazebo::event::Events::ConnectAddEntity(
            [this](std::string entity_name) { onAddEntity(std::move(entity_name)); });

    _delete_entity_event = gazebo::event::Events::ConnectDeleteEntity(
            [this](std::string entity_name) { onDeleteEntity(std::move(entity_name)); });

    // create logger and register log handler
    _logger = std::make_shared<vsm::Logger>();
    _logger->addLogHandler(
            static_cast<vsm::Logger::Level>(std::stoi(yamlField(_yaml, "verbosity"))),
            [](vsm::msecs time, vsm::Logger::Level level, vsm::Error error, const void* data,
                    size_t /*len*/) {
                std::cout << "VSM plugin: t " << time.count() << ", lv " << level << ", type "
                          << error.type << ", code " << error.code << ", msg " << error.what()
                          << " " << (error.type == SYNCED_ENTITY_ADDED ? (const char*) data : "")
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
    // update model pointers for synced entities in world
    for (auto& synced_entity : _synced_entities) {
        synced_entity.second.model = _world->ModelByName(synced_entity.first);
    }
    _logger->log(vsm::Logger::INFO, vsm::Error(STRERR(WORLD_CREATED)));

    auto models = _world->Models();
    for (auto model : models) {
        gazebo::msgs::Model msg;
        model->FillMsg(msg);
        // std::cout << msg.DebugString() << std::endl;
        std::cout << msg.name() << std::endl;
    }
}

void GazeboVsm::onWorldUpdateBegin(const common::UpdateInfo& update_info) {
    const auto vsm_entities_callback = [&](const vsm::EgoSphere::EntityLookup& vsm_entities) {
        for (const auto& vsm_entity : vsm_entities) {
            auto synced_entity = _synced_entities.find(vsm_entity.first);
            if (synced_entity == _synced_entities.end()) {
                // entity doesn't exist yet, create it
            }
            if (!synced_entity->second.model) {
                // entity was deleted, skip
                continue;
            }
            // parse message and update model
            gazebo::msgs::Model protobuf_msg;
            const auto& data = synced_entity->second.msg.data;
            protobuf_msg.ParseFromArray(data.data(), data.size());
            synced_entity->second.model->ProcessMsg(protobuf_msg);
        }
    };
    _mesh_node->readEntities(vsm_entities_callback);
}

void GazeboVsm::onWorldUpdateEnd() {
    // convert synced entities list to message and broadcast
    std::vector<vsm::EntityT> entity_msgs;
    entity_msgs.reserve(_synced_entities.size());
    for (auto synced_entity = _synced_entities.begin(); synced_entity != _synced_entities.end();) {
        entity_msgs.emplace_back(synced_entity->second.msg);
        if (synced_entity->second.model) {
            // serialize entity message
            gazebo::msgs::Model protobuf_msg;
            synced_entity->second.model->FillMsg(protobuf_msg);
            std::vector<uint8_t> buffer(protobuf_msg.ByteSize());
            protobuf_msg.SerializeToArray(buffer.data(), buffer.size());
            entity_msgs.back().data = std::move(buffer);
            entity_msgs.back().coordinates = getModelCoords(*synced_entity->second.model);
            ++synced_entity;
        } else {
            // delete entities that are zeroed out (but include expiry message in broadcast)
            synced_entity = _synced_entities.erase(synced_entity);
        }
    }
    _mesh_node->updateEntities(entity_msgs, true);
}

void GazeboVsm::onAddEntity(std::string entity_name) {
    // add entity to synced_entities if name pattern is matched
    for (const auto& synced_entity : _yaml["synced_entities"]) {
        if (std::regex_match(entity_name, std::regex(yamlField(synced_entity, "name_pattern")))) {
            vsm::EntityT entity_msg;
            entity_msg.name = entity_name;
            entity_msg.filter =
                    static_cast<vsm::Filter>(std::stoi(yamlField(synced_entity, "filter")));
            entity_msg.hop_limit = std::stoul(yamlField(synced_entity, "hop_limit"));
            entity_msg.range = std::stof(yamlField(synced_entity, "range"));
            entity_msg.expiry = std::stoul(yamlField(synced_entity, "expiry"));
            _synced_entities[entity_name] = {_world ? _world->ModelByName(entity_name) : nullptr,
                    std::move(entity_msg), synced_entity};
            _logger->log(vsm::Logger::INFO, vsm::Error(STRERR(SYNCED_ENTITY_ADDED)),
                    entity_name.c_str());
            break;
        }
    }
}

void GazeboVsm::onDeleteEntity(std::string entity_name) {
    auto synced_entity = _synced_entities.find(entity_name);
    // only process synced entities
    if (synced_entity == _synced_entities.end()) {
        return;
    }
    // zero out entity fields
    synced_entity->second.msg.coordinates = getModelCoords(*synced_entity->second.model);
    synced_entity->second.msg.expiry = 0;
    synced_entity->second.model = nullptr;
    _logger->log(vsm::Logger::INFO, vsm::Error(STRERR(SYNCED_ENTITY_DELETED)), entity_name.c_str());
}

std::vector<float> GazeboVsm::getModelCoords(const physics::Model& model) {
    const auto& pos = model.WorldPose().Pos();
    return {static_cast<float>(pos.X()), static_cast<float>(pos.Y()), static_cast<float>(pos.Z())};
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
