/**
* @file debugkeyvalue.cpp
* @author Caglar Yilmaz <yilmaz.caglar@tubitak.gov.tr>
*/

#include "debugkeyvalue.hpp"
#include <cstring> 			// For memset
#include <parameters/param.h>

const char *const UavcanDebugKeyValueBridge::NAME = "debug_key_value";

UavcanDebugKeyValueBridge::UavcanDebugKeyValueBridge(uavcan::INode &node) :
    UavcanSensorBridgeBase("uavcan_debug_key_value", ORB_ID(debug_key_value)),
    _sub_key_value_data(node)
{
}

int UavcanDebugKeyValueBridge::init() {
    int res = _sub_key_value_data.start(KeyValueCbBinder(this, &UavcanDebugKeyValueBridge::key_value_sub_cb));

    if (res < 0) {
        DEVICE_LOG("failed to start uavcan sub: %d", res);
        return res;
    }

    return 0;
}

void UavcanDebugKeyValueBridge::key_value_sub_cb(const uavcan::ReceivedDataStructure<uavcan::protocol::debug::KeyValue> &msg) {
    uint8_t node_id = msg.getSrcNodeID().get();

    auto report = debug_key_value_s();
    report.timestamp = hrt_absolute_time();
    memset(report.key, 0, sizeof(report.key));  // Null terminate

    // Extract first 3 characters
    for (int i = 0; i < 3; ++i) {
        report.key[i] = static_cast<char>(msg.key[i]);
    }
    report.value = msg.value;

    // Publish debug_key_value as before
    publish(node_id, &report);

    // Check and publish FlowrateRaw
    if (strncmp(report.key, "FM", 2) == 0) {
        _flowrate_report.timestamp = hrt_absolute_time();
        _flowrate_report.node_id = node_id;
        _flowrate_report.flowrate = msg.value;

        _to_flowrate_raw.publish(_flowrate_report);

    // Check and publish WeightRaw
    } else if (strncmp(report.key, "WH", 2) == 0) {
        _weight_report.timestamp = hrt_absolute_time();
        _weight_report.node_id = node_id;
        _weight_report.weight = msg.value;

        _to_weight_raw.publish(_weight_report);
    }
}


int UavcanDebugKeyValueBridge::init_driver(uavcan_bridge::Channel *channel) {
	return PX4_OK;
}
