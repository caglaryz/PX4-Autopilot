/**
* @file debugkeyvalue.hpp
* @author Caglar Yilmaz <yilmaz.caglar@tubitak.gov.tr>
* @brief DroneCAN reception for Debug Key Value messages with added FlowrateRaw and WeightRaw uORB updates.
*/

#pragma once

#include "sensor_bridge.hpp"
#include <uavcan/protocol/debug/KeyValue.hpp>  // UAVCAN DSDL
#include <uORB/topics/debug_key_value.h>       // Existing uORB topic
#include <uORB/topics/flowrate_raw.h>          // New FlowrateRaw uORB topic
#include <uORB/topics/weight_raw.h>            // New WeightRaw uORB topic
#include <uORB/Publication.hpp>

class UavcanDebugKeyValueBridge : public UavcanSensorBridgeBase {
public:
    static const char *const NAME;

    UavcanDebugKeyValueBridge(uavcan::INode &node);

    const char *get_name() const override { return NAME; }

    int init() override;

private:
    void key_value_sub_cb(const uavcan::ReceivedDataStructure<uavcan::protocol::debug::KeyValue> &msg);
    int init_driver(uavcan_bridge::Channel *channel) override;

    typedef uavcan::MethodBinder<UavcanDebugKeyValueBridge *,
                                 void (UavcanDebugKeyValueBridge::*) (const uavcan::ReceivedDataStructure<uavcan::protocol::debug::KeyValue>&)>
        KeyValueCbBinder;

    uavcan::Subscriber<uavcan::protocol::debug::KeyValue, KeyValueCbBinder> _sub_key_value_data;

    // Declare the uORB Publications properly
    uORB::Publication<flowrate_raw_s> _to_flowrate_raw{ORB_ID(flowrate_raw)};
    uORB::Publication<weight_raw_s> _to_weight_raw{ORB_ID(weight_raw)};

    // Structs for holding uORB message data
    flowrate_raw_s _flowrate_report{};
    weight_raw_s _weight_report{};

};
