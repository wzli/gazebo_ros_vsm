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
    GazeboVsm() {}
    void Load(int /*_argc*/, char** /*_argv*/);

    void onWorldCreated(std::string world_name);

private:
    std::string yamlString(const YAML::Node& node) const;

    gazebo::physics::WorldPtr _world;

    gazebo::event::ConnectionPtr _world_created_event;

    YAML::Node _yaml;
    std::unique_ptr<vsm::MeshNode> _mesh_node;
};

}  // namespace gazebo
