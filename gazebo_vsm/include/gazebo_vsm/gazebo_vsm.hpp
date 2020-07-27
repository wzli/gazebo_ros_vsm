#pragma once
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <vsm/mesh_node.hpp>
#include <yaml-cpp/yaml.h>
#include <memory>
#include <unordered_map>

namespace gazebo {

class GazeboVsm : public SystemPlugin {
public:
    enum LogCode {
        START_OFFSET = 1000,
        WORLD_CREATED,
        SYNCED_ENTITY_ADDED,
        SYNCED_ENTITY_DELETED,
    };

    void Load(int argc, char** argv);

    void onWorldCreated(std::string world_name);
    void onWorldUpdateBegin(const common::UpdateInfo& update_info);
    void onWorldUpdateEnd();
    void onAddEntity(std::string entity_name);
    void onDeleteEntity(std::string entity_name);

private:
    struct SyncedEntity {
        physics::ModelPtr model;
        vsm::EntityT msg;
        YAML::Node yaml;
    };

    void initMeshNode();

    std::string yamlField(YAML::Node node, std::string field, bool required = true) const;

    physics::WorldPtr _world;

    event::ConnectionPtr _world_created_event;
    event::ConnectionPtr _world_update_begin_event;
    event::ConnectionPtr _world_update_end_event;
    event::ConnectionPtr _add_entity_event;
    event::ConnectionPtr _delete_entity_event;

    YAML::Node _yaml;
    std::unique_ptr<vsm::MeshNode> _mesh_node;
    std::shared_ptr<vsm::Logger> _logger;

    std::unordered_map<std::string, SyncedEntity> _synced_entities;
};

}  // namespace gazebo
