#pragma once
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <vsm/mesh_node.hpp>
#include <vsm/zmq_transport.hpp>
#include <yaml-cpp/yaml.h>
#include <pugixml.hpp>
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
        MODEL_STATE_SDF_PARSE_FAIL,
        TRACKED_ENTITY_FOUND,
        BOOTSTRAP_PEER_ADDED,
        REMOTE_ENTITY_DELETED,

        // req/rep codes
        SOCKET_CONNECT_FAIL,
        MESSAGE_VERIFY_FAIL,
        REQUEST_MISSING_SOURCE,
        REQUEST_NONEXISTING_ENTITY,
        REQUEST_SENT,
        REQUEST_RESPONSE_SENT,
        REQUEST_RESPONSE_RECEIVED,
        REQUEST_RESPONSE_MISMATCH,
        REQUEST_RESPONSE_REJECTED,
    };

    void Load(int argc, char** argv);

    void onWorldCreated(std::string world_name);
    void onWorldUpdateBegin(const common::UpdateInfo& update_info);
    void onWorldUpdateEnd();
    void onAddEntity(std::string entity_name);

    bool onVsmUpdate(vsm::EgoSphere::EntityUpdate* new_entity,
            const vsm::EgoSphere::EntityUpdate* old_entity, const vsm::NodeInfoT& source);

    void onVsmReqMsg(const void* buffer, size_t len);
    void onVsmRepMsg(const void* buffer, size_t len);

    const vsm::NodeInfo* parseMsg(const void* buffer, size_t len) const;
    bool sendMsg(vsm::NodeInfoT msg, const char* topic);

private:
    struct SyncedEntity {
        physics::ModelPtr model;
        vsm::EntityT msg;
        YAML::Node yaml;
    };

    struct EntityRequest {
        std::string source;
        std::string name;
        std::string sdf;
    };

    void initMeshNode();
    bool parseModelState(physics::ModelState& model_state, const void* data, size_t len);
    static void parseSdf(const pugi::xml_node& node, sdf::ElementPtr sdf);
    static std::vector<float> getEntityCoords(const physics::Entity& model);
    std::string envSubstitute(std::string str, bool required = true) const;
    std::string yamlField(YAML::Node node, std::string field, bool required = true) const;

    physics::WorldPtr _world;
    physics::EntityPtr _tracked_entity;
    physics::ModelState _model_state;
    sdf::ElementPtr _model_state_sdf;

    event::ConnectionPtr _world_created_event;
    event::ConnectionPtr _world_update_begin_event;
    event::ConnectionPtr _world_update_end_event;
    event::ConnectionPtr _add_entity_event;

    // disabled events
    event::ConnectionPtr _pause_event;
    event::ConnectionPtr _step_event;
    event::ConnectionPtr _stop_event;
    event::ConnectionPtr _time_reset_event;
    event::ConnectionPtr _world_reset_event;

    YAML::Node _yaml;
    std::unique_ptr<vsm::MeshNode> _mesh_node;
    std::shared_ptr<vsm::Logger> _logger;
    std::unique_ptr<zmq::socket_t> _tx_socket;

    std::unordered_map<std::string, SyncedEntity> _synced_entities;
    std::unordered_map<uint32_t, EntityRequest> _entity_requests;
    uint32_t _entity_request_count;

    std::vector<std::string> _added_entities;
    std::vector<std::string> _deleted_entities;
    std::vector<std::string> _prev_deleted_entities;
    vsm::msecs _prev_msg_ts;
};

}  // namespace gazebo
