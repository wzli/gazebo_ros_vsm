#pragma once
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>

namespace gazebo {

class GazeboVsm : public SystemPlugin {
public:
    GazeboVsm() {}
    void Load(int /*_argc*/, char** /*_argv*/);

    void onWorldCreated(std::string world_name);

private:
    gazebo::physics::WorldPtr _world;

    gazebo::event::ConnectionPtr _world_created_event;
};

}  // namespace gazebo
