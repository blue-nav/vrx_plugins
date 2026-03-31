/*
 * Copyright (C) 2025 BlueNav
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <chrono>
#include <functional>
#include <queue>
#include <string>
#include <mutex>
#include <utility>
#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/Factory.hh>
#include <gz/msgs/navsat.pb.h>
#include <gz/msgs/imu.pb.h>
#include <sdf/sdf.hh>

#include "SensorLatency.hh"

using namespace gz;
using namespace vrx;

/// \brief Private SensorLatency data class.
class vrx::SensorLatency::Implementation
{
  /// \brief Structure to hold a delayed message with its publish time.
  /// Using std::pair for simplicity (time, message).
  public: using DelayedMessage = std::pair<double, transport::ProtoMsgPtr>;

  /// \brief A transport node.
  public: transport::Node node;

  /// \brief Input topic name (sensor output).
  public: std::string inputTopic;

  /// \brief Output topic name (delayed messages).
  public: std::string outputTopic;

  /// \brief Message type name.
  public: std::string messageType;

  /// \brief Latency in seconds.
  public: double latency = 0.0;

  /// \brief Update rate of the sensor (Hz). Used to calculate maxQueueSize.
  public: double updateRate = 0.0;

  /// \brief Maximum queue size (auto-calculated from update_rate and latency).
  public: size_t maxQueueSize = 100;

  /// \brief Publisher for delayed messages.
  public: transport::Node::Publisher outputPub;

  /// \brief Priority queue of delayed messages (earliest publish time first).
  /// Using greater<> comparator so earliest time has highest priority.
  public: std::priority_queue<DelayedMessage, std::vector<DelayedMessage>,
                              std::greater<DelayedMessage>> messageQueue;

  /// \brief Mutex for thread-safe access to message queue.
  public: std::mutex queueMutex;

  /// \brief Last known simulation time (updated in PostUpdate).
  public: double lastSimTime = 0.0;

  /// \brief Subscribe to the input topic with the correct message type.
  /// \return True if subscription succeeded.
  public: bool SubscribeToTopic();

};

//////////////////////////////////////////////////
SensorLatency::SensorLatency()
  : System(), dataPtr(utils::MakeUniqueImpl<Implementation>())
{
}

//////////////////////////////////////////////////
// Helper to get SDF element
template<typename T>
bool GetSdfElement(const std::shared_ptr<const sdf::Element> &_sdf,
                   const std::string &_name, T &_value, bool _required = true)
{
  if (!_sdf->HasElement(_name))
  {
    if (_required)
      gzerr << "SensorLatency: Missing required <" << _name << "> element" << std::endl;
    return false;
  }
  _value = _sdf->Get<T>(_name);
  return true;
}

//////////////////////////////////////////////////
void SensorLatency::Configure(const sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    sim::EntityComponentManager &_ecm,
    sim::EventManager &/*_eventMgr*/)
{
  auto &d = *this->dataPtr;
  
  // Get required parameters
  if (!GetSdfElement(_sdf, "input_topic", d.inputTopic) ||
      !GetSdfElement(_sdf, "output_topic", d.outputTopic) ||
      !GetSdfElement(_sdf, "message_type", d.messageType) ||
      !GetSdfElement(_sdf, "latency", d.latency) ||
      !GetSdfElement(_sdf, "update_rate", d.updateRate))
    return;

  // Normalize and validate
  d.inputTopic = transport::TopicUtils::AsValidTopic(d.inputTopic);
  d.outputTopic = transport::TopicUtils::AsValidTopic(d.outputTopic);
  d.latency = std::max(0.0, d.latency);
  
  if (d.updateRate <= 0.0)
  {
    gzerr << "SensorLatency: update_rate must be positive (got: " << d.updateRate << ")" << std::endl;
    return;
  }
  // Calculate max queue size: ceil(update_rate * latency * 3.0), minimum 10
  d.maxQueueSize = std::max(size_t(10), static_cast<size_t>(std::ceil(d.updateRate * d.latency * 3.0)));

  // Initialize publisher and subscriber
  d.outputPub = d.node.Advertise(d.outputTopic, d.messageType);
  if (!d.outputPub.Valid() || !d.SubscribeToTopic())
  {
    gzerr << "SensorLatency: Failed to initialize (topic: " << d.outputTopic
          << ", type: " << d.messageType << ")" << std::endl;
    return;
  }

  gzmsg << "SensorLatency: Configured latency=" << d.latency
        << "s, update_rate=" << d.updateRate << "Hz, maxQueueSize=" << d.maxQueueSize
        << ", input=" << d.inputTopic << ", output=" << d.outputTopic << std::endl;
}

//////////////////////////////////////////////////
// Template helper to subscribe with typed callback
template<typename T>
bool SubscribeWithType(transport::Node &_node, const std::string &_topic,
                       vrx::SensorLatency::Implementation *_impl)
{
  return _node.Subscribe(_topic, std::function<void(const T &)>([=](const T &_msg)
  {
    std::lock_guard<std::mutex> lock(_impl->queueMutex);
    if (_impl->messageQueue.size() >= _impl->maxQueueSize)
      _impl->messageQueue.pop();
    _impl->messageQueue.emplace(_impl->lastSimTime + _impl->latency, std::make_shared<T>(_msg));
  }));
}

//////////////////////////////////////////////////
bool SensorLatency::Implementation::SubscribeToTopic()
{
  if (messageType == "gz.msgs.NavSat")
    return SubscribeWithType<msgs::NavSat>(node, inputTopic, this);
  if (messageType == "gz.msgs.IMU")
    return SubscribeWithType<msgs::IMU>(node, inputTopic, this);
  gzerr << "SensorLatency: Unsupported message type [" << messageType
        << "]. Supported: gz.msgs.NavSat, gz.msgs.IMU" << std::endl;
  return false;
}

//////////////////////////////////////////////////
void SensorLatency::PostUpdate(
    const sim::UpdateInfo &_info,
    const sim::EntityComponentManager &/*_ecm*/)
{
  GZ_PROFILE("SensorLatency::PostUpdate");
  if (_info.paused)
    return;

  auto &d = *this->dataPtr;
  std::lock_guard<std::mutex> lock(d.queueMutex);
  d.lastSimTime = std::chrono::duration<double>(_info.simTime).count();
  
  while (!d.messageQueue.empty() && d.messageQueue.top().first <= d.lastSimTime)
  {
    d.outputPub.Publish(*d.messageQueue.top().second);
    d.messageQueue.pop();
  }
}

GZ_ADD_PLUGIN(SensorLatency,
              sim::System,
              SensorLatency::ISystemConfigure,
              SensorLatency::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(vrx::SensorLatency,
                    "vrx::SensorLatency")

