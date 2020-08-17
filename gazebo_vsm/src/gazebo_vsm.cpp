#include <gazebo_vsm/gazebo_vsm.hpp>
#include <gazebo_vsm/bzip2.hpp>

#include <algorithm>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <regex>

// print backtrace on fault
#ifdef __linux__
#include <execinfo.h>
#include <signal.h>

static void fault_handler(int sig) {
    void* array[20];
    size_t size;
    size = backtrace(array, 20);
    fprintf(stderr, "Error: signal %d:\n", sig);
    backtrace_symbols_fd(array, size, STDERR_FILENO);
    exit(1);
}
#endif

namespace gazebo {

void GazeboVsm::Load(int argc, char** argv) {
#ifdef __linux__
    // set fault handlers
    signal(SIGSEGV, fault_handler);
    signal(SIGABRT, fault_handler);
#endif
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

    // throw error on disabled events
    //_pause_event = gazebo::event::Events::ConnectPause(
    //        [](bool) { throw std::runtime_error("VSM plugin: pause event disabled"); });

    _step_event = gazebo::event::Events::ConnectStep(
            []() { throw std::runtime_error("VSM plugin: step event disabled"); });

    _stop_event = gazebo::event::Events::ConnectStop(
            []() { throw std::runtime_error("VSM plugin: stop event disabled"); });

    _time_reset_event = gazebo::event::Events::ConnectTimeReset(
            []() { throw std::runtime_error("VSM plugin: time reset event disabled"); });

    _world_reset_event = gazebo::event::Events::ConnectWorldReset(
            []() { throw std::runtime_error("VSM plugin: world reset event disabled"); });

    // create logger and register log handler
    _logger = std::make_shared<vsm::Logger>();
    _logger->addLogHandler(
            static_cast<vsm::Logger::Level>(std::stoi(yamlField(_yaml, "verbosity"))),
            [](vsm::msecs time, vsm::Logger::Level level, vsm::Error error, const void* data,
                    size_t /*len*/) {
                std::cout << "VSM plugin: t " << time.count() << ", lv " << level << ", type "
                          << error.type << ", code " << error.code << ", msg " << error.what();
                switch (error.type) {
                    // fall through
                    case WORLD_CREATED:
                    case SYNCED_ENTITY_ADDED:
                    case SYNCED_ENTITY_DELETED:
                    case TRACKED_ENTITY_FOUND:
                    case MODEL_STATE_SDF_PARSE_FAIL:
                    case BOOTSTRAP_PEER_ADDED:
                    case VSM_ENTITY_DELETED:
                    case REQUEST_SENT:
                    case REQUEST_RESPONSE_SENT:
                    case REQUEST_RESPONSE_RECEIVED:
                    case REQUEST_NONEXISTING_ENTITY:
                        std::cout << " " << static_cast<const char*>(data);
                    // fall through
                    default:
                        std::cout << std::endl;
                }
            });
    // create mesh node
    initVsm();
}

void GazeboVsm::initVsm() {
    _protocol = yamlField(_yaml, "protocol");
    auto zmq_transport =
            std::make_shared<vsm::ZmqTransport>(_protocol + "://*:" + yamlField(_yaml, "port"));
    _mesh_node = std::make_unique<vsm::MeshNode>(vsm::MeshNode::Config{
            vsm::msecs(std::stoul(yamlField(_yaml, "peer_update_interval"))),
            vsm::msecs(std::stoul(yamlField(_yaml, "entity_expiry_interval"))),
            std::stoul(yamlField(_yaml, "entity_updates_size")),
            // ego sphere
            {
                    [this](vsm::EgoSphere::EntityUpdate* new_entity,
                            const vsm::EgoSphere::EntityUpdate* old_entity,
                            const vsm::NodeInfoT& source) {
                        return onVsmUpdate(new_entity, old_entity, source);
                    },
                    std::stoul(yamlField(_yaml, "timestamp_lookup_size")),
            },
            // peer tracker
            {
                    yamlField(_yaml, "name", false),
                    _protocol + "://" + yamlField(_yaml, "address") + ":" +
                            yamlField(_yaml, "port"),
                    _yaml["initial_coordinates"]
                            ? _yaml["initial_coordinates"].as<std::vector<float>>()
                            : std::vector<float>(3),
                    std::stof(yamlField(_yaml, "power_radius")),
                    std::stoul(yamlField(_yaml, "connection_degree")),
                    std::stoul(yamlField(_yaml, "peer_lookup_size")),
                    std::stof(yamlField(_yaml, "peer_rank_decay")),
            },
            zmq_transport, _logger,
            // std::function<msecs(void)> local_clock
    });

    // register request and response message handlers
    if (zmq_transport->addReceiver(
                [this](const void* buffer, size_t len) { onVsmReqMsg(buffer, len); }, "req") ||
            zmq_transport->addReceiver(
                    [this](const void* buffer, size_t len) { onVsmRepMsg(buffer, len); }, "rep")) {
        throw std::runtime_error("VSM plugin: failed to initalize req/rep handlers.");
    }

    // inject bootstrap peers
    for (const auto& bootstrap_peer : _yaml["bootstrap_peers"]) {
        auto peer_endpoint = envSubstitute(bootstrap_peer.as<std::string>(), false);
        if (peer_endpoint.empty() || peer_endpoint == " ") {
            continue;
        }
        _mesh_node->getPeerTracker().latchPeer((_protocol + "://" + peer_endpoint).c_str(), 1);
        _logger->log(vsm::Logger::INFO, vsm::Error(STRERR(BOOTSTRAP_PEER_ADDED)),
                peer_endpoint.c_str(), peer_endpoint.size());
    }

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
    _logger->log(vsm::Logger::INFO, vsm::Error(STRERR(WORLD_CREATED)), world_name.c_str());
    // find tracked entity if configured
    auto tracked_name = yamlField(_yaml, "tracked_entity", false);
    _tracked_entity = _world->EntityByName(tracked_name);
    if (_tracked_entity) {
        _logger->log(
                vsm::Logger::INFO, vsm::Error(STRERR(TRACKED_ENTITY_FOUND)), tracked_name.c_str());
    }
}

void GazeboVsm::onWorldUpdateBegin(const common::UpdateInfo&) {
    // skip first round before there is anything to sync
    if (!_model_state_sdf) {
        auto states_sdf = _world->SDF()->GetElement("state");
        states_sdf->ClearElements();
        _model_state_sdf = states_sdf->AddElement("model");
        return;
    }
    // vsm entities are mutexed locked inside the below callback
    _mesh_node->readEntities([&](const vsm::EgoSphere::EntityLookup& vsm_entities) {
        // sort added entities list for faster lookup
        std::sort(_added_entities.begin(), _added_entities.end());
        // update syncronized entities with latest vsm updates
        for (const auto& synced_entity : _synced_entities) {
            // skip if entity was already deleted
            if (!synced_entity.second.model || !synced_entity.second.model->GetWorld()) {
                continue;
            }
            // look for entity in vsm egosphere
            auto vsm_entity = vsm_entities.find(synced_entity.first);
            // deleted entity if not found in vsm egosphere and not recently added
            if (vsm_entity == vsm_entities.end()) {
                if (!std::binary_search(
                            _added_entities.begin(), _added_entities.end(), synced_entity.first)) {
                    _world->RemoveModel(synced_entity.second.model);
                    _logger->log(vsm::Logger::INFO, vsm::Error(STRERR(VSM_ENTITY_DELETED)),
                            synced_entity.first.c_str());
                }
                continue;
            }
            // skip sync if entity update was self generated
            if (!vsm_entity->second.hops) {
                continue;
            }
            // parse message and update model
            const auto& data = vsm_entity->second.entity.data;
            auto model_state_sdf = bzip2::decompress(std::string(data.begin(), data.end()));
            if (parseModelState(_model_state, model_state_sdf.c_str(), model_state_sdf.size())) {
                synced_entity.second.model->SetState(_model_state);
                // std::cout << "Parsed:\r\n" << _model_state_sdf->ToString("") << std::endl;
            }
        }
        _added_entities.clear();

        // request for full SDF of new entities broadcasted by other nodes
        for (auto req = _entity_requests.begin(); req != _entity_requests.end();) {
            if (_synced_entities.find(req->second.name) != _synced_entities.end()) {
                // delete request if entity already exist
                req = _entity_requests.erase(req);
            } else if (!req->second.sdf.empty()) {
                // create model if response sdf is received
                _world->InsertModelString("<sdf version='1.6'>" + req->second.sdf + "</sdf>");
                _logger->log(vsm::Logger::INFO,
                        vsm::Error(STRERR(REQUEST_RESPONSE_RECEIVED), req->first),
                        req->second.name.c_str());
                // std::cout << "got req: \r\n" << req->second.sdf << std::endl;
                req = _entity_requests.erase(req);
            } else {
                // send request again otherwise
                vsm::NodeInfoT msg;
                msg.address = req->second.source;
                msg.name = req->second.name;
                msg.sequence = req->first;
                sendMsg(std::move(msg), "req");
                _logger->log(vsm::Logger::INFO, vsm::Error(STRERR(REQUEST_SENT), req->first),
                        req->second.name.c_str());
                ++req;
            }
        }
    });
}

void GazeboVsm::onWorldUpdateEnd() {
    // update mesh node pose based on tracked entity
    if (_tracked_entity && _tracked_entity->GetWorld()) {
        _mesh_node->getPeerTracker().getNodeInfo().coordinates = getEntityCoords(*_tracked_entity);
    }
    // convert synced entities list to message and broadcast
    std::vector<vsm::EntityT> entity_msgs;
    entity_msgs.reserve(_synced_entities.size());
    for (auto synced_entity = _synced_entities.begin(); synced_entity != _synced_entities.end();) {
        // skip invalid entity
        if (!synced_entity->second.model) {
            ++synced_entity;
            continue;
        }
        // delete entities that are zeroed out (but include expiry message in broadcast)
        if (!synced_entity->second.model->GetWorld()) {
            entity_msgs.emplace_back(synced_entity->second.msg);
            entity_msgs.back().expiry = 0;
            entity_msgs.back().coordinates = getEntityCoords(*synced_entity->second.model);
            _logger->log(vsm::Logger::INFO, vsm::Error(STRERR(SYNCED_ENTITY_DELETED)),
                    synced_entity->first.c_str());
            synced_entity = _synced_entities.erase(synced_entity);
            continue;
        }
        // don't broadcast entity if filter is ALL (to avoid conficts with source)
        if (synced_entity->second.msg.filter == vsm::Filter::ALL) {
            ++synced_entity;
            continue;
        }
        // serialize entity message
        physics::ModelState model_state(synced_entity->second.model);
        _model_state_sdf->ClearElements();
        model_state.FillSDF(_model_state_sdf);
        auto model_sdf_str = bzip2::compress(_model_state_sdf->ToString(""));
        entity_msgs.emplace_back(synced_entity->second.msg);
        entity_msgs.back().data.assign(model_sdf_str.begin(), model_sdf_str.end());
        entity_msgs.back().coordinates = getEntityCoords(*synced_entity->second.model);
        ++synced_entity;
    }
    // broadcast new entity updates
    auto msgs = _mesh_node->updateEntities(entity_msgs, true);
}

void GazeboVsm::onAddEntity(std::string entity_name) {
    // scan for name remappings
    auto remapped_entities = _yaml["remapped_entities"];
    if (remapped_entities) {
        auto remap = yamlField(remapped_entities, entity_name, false);
        if (!remap.empty()) {
            boost::dynamic_pointer_cast<physics::Model>(_world->BaseByName(entity_name))
                    ->SetName(remap);
            entity_name = remap;
        }
    }
    // match tracked entity if configured
    if (!_tracked_entity && _world && entity_name == yamlField(_yaml, "tracked_entity", false)) {
        _tracked_entity =
                boost::dynamic_pointer_cast<physics::Model>(_world->BaseByName(entity_name));
        _logger->log(
                vsm::Logger::INFO, vsm::Error(STRERR(TRACKED_ENTITY_FOUND)), entity_name.c_str());
    }
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
            _synced_entities[entity_name] = {_world ? boost::dynamic_pointer_cast<physics::Model>(
                                                              _world->BaseByName(entity_name))
                                                    : nullptr,
                    std::move(entity_msg), synced_entity};
            _added_entities.push_back(entity_name);
            _logger->log(vsm::Logger::INFO, vsm::Error(STRERR(SYNCED_ENTITY_ADDED)),
                    entity_name.c_str());
            break;
        }
    }
}

bool GazeboVsm::onVsmUpdate(vsm::EgoSphere::EntityUpdate* new_entity,
        const vsm::EgoSphere::EntityUpdate* old_entity, const vsm::NodeInfoT& source) {
    // add to deleted entities
    if (!new_entity) {
        _deleted_entities.push_back(old_entity->entity.name);
        return true;
    }
    // reset deleted entities list every time self simulation updates
    if (new_entity->hops == 0 && new_entity->source_timestamp != _prev_msg_ts) {
        _prev_msg_ts = new_entity->source_timestamp;
        std::sort(_deleted_entities.begin(), _deleted_entities.end());
        _prev_deleted_entities.swap(_deleted_entities);
        _deleted_entities.clear();
    }
    if (!old_entity) {
        // only allow entity creation if entity wasn't deleted in the previous round
        if (std::binary_search(_prev_deleted_entities.begin(), _prev_deleted_entities.end(),
                    new_entity->entity.name)) {
            return false;
        }
        // if new entity has a remote source, request for sdf inorder to spawn locally
        if (new_entity->hops != 0) {
            EntityRequest req{source.address, new_entity->entity.name, "", {}};
            _entity_requests.emplace(++_entity_request_count, std::move(req));
            return true;
        }
    }
    return true;
}

void GazeboVsm::onVsmReqMsg(const void* buffer, size_t len) {
    const vsm::NodeInfo* msg = parseMsg(buffer, len);
    if (!msg) {
        return;
    }
    if (!msg->address()) {
        _logger->log(vsm::Logger::WARN, vsm::Error(STRERR(REQUEST_MISSING_SOURCE)), msg, len);
        return;
    }
    vsm::NodeInfoT rep;
    msg->UnPackTo(&rep);
    auto synced_entity = _synced_entities.find(rep.name);
    // reject request for non-existing entity
    if (synced_entity == _synced_entities.end() || !synced_entity->second.model ||
            !synced_entity->second.model->GetWorld()) {
        rep.name.clear();
        _logger->log(vsm::Logger::WARN, vsm::Error(STRERR(REQUEST_NONEXISTING_ENTITY)), msg, len);
        sendMsg(std::move(rep), "rep");
        return;
    }
    auto entity_sdf = synced_entity->second.model->GetSDF();
    remove_plugins(entity_sdf);
    auto data = bzip2::compress(entity_sdf->ToString(""));
    const int chunks = data.size() / 1024 + ((data.size() % 1024) > 0);
    rep.coordinates = {0, static_cast<float>(chunks)};
    auto chunk_end = data.begin() + std::min<int>(data.size(), 1024);
    for (auto chunk_start = data.begin(); chunk_start != data.end(); chunk_start = chunk_end) {
        chunk_end = chunk_start + std::min<int>(std::distance(chunk_start, data.end()), 1024);
        rep.name = std::string(chunk_start, chunk_end);
        sendMsg(rep, "rep");
        ++rep.coordinates[0];
    }
    _logger->log(vsm::Logger::INFO, vsm::Error(STRERR(REQUEST_RESPONSE_SENT), msg->sequence()),
            msg->name()->c_str());
}

void GazeboVsm::onVsmRepMsg(const void* buffer, size_t len) {
    const vsm::NodeInfo* msg = parseMsg(buffer, len);
    if (!msg) {
        return;
    }
    auto entity_request = _entity_requests.find(msg->sequence());
    if (entity_request == _entity_requests.end() ||
            entity_request->second.source != msg->address()->c_str()) {
        _logger->log(vsm::Logger::WARN,
                vsm::Error(STRERR(REQUEST_RESPONSE_MISMATCH), msg->sequence()), msg, len);
        return;
    }
    if (!msg->coordinates() || msg->coordinates()->size() != 2 ||
            msg->coordinates()->Get(0) >= msg->coordinates()->Get(1) || !msg->name()) {
        _logger->log(vsm::Logger::WARN,
                vsm::Error(STRERR(REQUEST_RESPONSE_REJECTED), msg->sequence()), msg, len);
        return;
    }
    auto& chunks = entity_request->second.chunks;
    chunks.resize(msg->coordinates()->Get(1));
    chunks[msg->coordinates()->Get(0)] = msg->name()->str();
    if (std::find(chunks.begin(), chunks.end(), std::string()) == chunks.end()) {
        std::string data;
        for (const auto& chunk : chunks) {
            data += chunk;
        }
        try {
            entity_request->second.sdf = bzip2::decompress(data);
        } catch (const std::exception& e) {
            _logger->log(vsm::Logger::WARN,
                    vsm::Error(STRERR(MESSAGE_DECOMPRESS_FAIL), msg->sequence()), msg, len);
            std::cout << e.what();
        }
    }
}

int GazeboVsm::sendMsg(vsm::NodeInfoT msg, const char* topic) {
    std::string dst_address = _mesh_node->getPeerTracker().getNodeInfo().address;
    dst_address.swap(msg.address);
    auto& transport = _mesh_node->getTransport();
    for (const auto& peer : _mesh_node->getConnectedPeers()) {
        if (peer != dst_address) {
            transport.disconnect(peer);
        }
    }
    flatbuffers::FlatBufferBuilder fbb;
    fbb.Finish(vsm::NodeInfo::Pack(fbb, &msg));
    int send_error = transport.transmit(fbb.GetBufferPointer(), fbb.GetSize(), topic);
    // printf("send error %d %s\r\n", send_error, zmq_strerror(send_error));
    for (const auto& peer : _mesh_node->getConnectedPeers()) {
        if (peer != dst_address) {
            transport.connect(peer);
        }
    }
    return send_error;
}

const vsm::NodeInfo* GazeboVsm::parseMsg(const void* buffer, size_t len) const {
    auto msg = flatbuffers::GetRoot<vsm::NodeInfo>(buffer);
    flatbuffers::Verifier verifier(static_cast<const uint8_t*>(buffer), len);
    if (!msg->Verify(verifier)) {
        _logger->log(vsm::Logger::WARN, vsm::Error(STRERR(MESSAGE_VERIFY_FAIL)), buffer, len);
        return nullptr;
    }
    return msg;
}

bool GazeboVsm::parseModelState(physics::ModelState& model_state, const void* data, size_t len) {
    pugi::xml_document dom;
    auto parse_result = dom.load_buffer(data, len);
    auto root = dom.child("model");
    if (!parse_result || !root) {
        _logger->log(vsm::Logger::WARN,
                vsm::Error(STRERR(MODEL_STATE_SDF_PARSE_FAIL), parse_result.status), data, len);
        return false;
    }
    _model_state_sdf->ClearElements();
    parseSdf(root, _model_state_sdf);
    model_state.Load(_model_state_sdf);
    return true;
}

void GazeboVsm::parseSdf(const pugi::xml_node& node, sdf::ElementPtr sdf) {
    if (*node.child_value()) {
        sdf->Set(node.child_value());
    }
    for (const auto& attr : node.attributes()) {
        sdf->GetAttribute(attr.name())->Set(attr.value());
    }
    for (const auto& elem : node.children()) {
        if (*elem.name()) {
            parseSdf(elem, sdf->AddElement(elem.name()));
        }
    }
}

void GazeboVsm::remove_plugins(sdf::ElementPtr elem) {
    for (auto child = elem->GetFirstElement(); child; child = child->GetNextElement()) {
        if (child->GetName() == "plugin") {
            child->RemoveFromParent();
        } else {
            remove_plugins(child);
        }
    }
}

std::vector<float> GazeboVsm::getEntityCoords(const physics::Entity& model) {
    const auto& pos = model.WorldPose().Pos();
    return {static_cast<float>(pos.X()), static_cast<float>(pos.Y()), static_cast<float>(pos.Z())};
}

std::string GazeboVsm::envSubstitute(std::string str, bool required) const {
    if (str.front() == '$') {
        const char* env = std::getenv(str.c_str() + 1);
        if (required && (!env || !*env)) {
            throw std::runtime_error("VSM plugin: missig required enviornment variable " + str);
        }
        str = env ? env : "";
    }
    return str;
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
    return envSubstitute(scalar.as<std::string>(), required);
}

GZ_REGISTER_SYSTEM_PLUGIN(GazeboVsm)

}  // namespace gazebo
