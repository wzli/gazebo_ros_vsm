#pragma once
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <vsm/mesh_node.hpp>
#include <yaml-cpp/yaml.h>
#include <memory>

namespace gazebo {

class GazeboVsm : public SystemPlugin {
public:
    enum LogCode {
        START_OFFSET = 1000,
        WORLD_CREATED,
    };

    GazeboVsm() {}
    void Load(int argc, char** argv);

    void onWorldCreated(std::string world_name);

private:
    void initMeshNode();

    std::string yamlField(YAML::Node node, std::string field, bool required = true) const;

    gazebo::physics::WorldPtr _world;
    gazebo::event::ConnectionPtr _world_created_event;

    gazebo::event::ConnectionPtr _add_entity_event;
    gazebo::event::ConnectionPtr _delete_entity_event;

    YAML::Node _yaml;
    std::unique_ptr<vsm::MeshNode> _mesh_node;
    std::shared_ptr<vsm::Logger> _logger;
};

}  // namespace gazebo
