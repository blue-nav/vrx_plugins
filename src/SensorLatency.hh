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

#ifndef VRX_SENSORLATENCY_HH_
#define VRX_SENSORLATENCY_HH_

#include <gz/sim/System.hh>
#include <gz/utils/ImplPtr.hh>
#include <sdf/sdf.hh>

namespace vrx
{
  /// \brief A system to add configurable latency to sensor messages.
  /// This plugin subscribes to a sensor topic, stores incoming messages,
  /// and republishes them after a specified delay.
  ///
  /// ## Required system parameters
  ///
  /// * `<input_topic>` - The topic name to subscribe to (sensor output).
  /// * `<output_topic>` - The topic name to publish delayed messages to.
  /// * `<message_type>` - The protobuf message type (e.g., `gz.msgs.NavSat`).
  /// * `<latency>` - The delay in seconds before republishing messages.
  /// * `<update_rate>` - Sensor update rate in Hz. Required for calculating queue size.
  ///                     The maximum queue size is automatically calculated as:
  ///                     max_queue_size = ceil(update_rate * latency * 3.0)
  ///                     Older messages are dropped if queue is full.
  ///
  /// ## Example
  /// <plugin filename="libSensorLatency.so" name="vrx::SensorLatency">
  ///   <input_topic>blueboat/sensors/gps_raw</input_topic>
  ///   <output_topic>blueboat/sensors/gps</output_topic>
  ///   <message_type>gz.msgs.NavSat</message_type>
  ///   <latency>0.1</latency>
  ///   <update_rate>20</update_rate>
  ///   <!-- max_queue_size auto-calculated as: ceil(20 * 0.1 * 3) = 6 -->
  /// </plugin>
  class SensorLatency
      : public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPostUpdate
  {
    /// \brief Constructor.
    public: SensorLatency();

    /// \brief Destructor.
    public: ~SensorLatency() override = default;

    // Documentation inherited.
    public: void Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventMgr) override;

    // Documentation inherited.
    public: void PostUpdate(
                const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer.
    GZ_UTILS_UNIQUE_IMPL_PTR(dataPtr)
  };
}

#endif

