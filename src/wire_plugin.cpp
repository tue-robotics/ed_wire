#include "ed_wire/wire_plugin.h"

#include <ed/entity.h>
#include <ed/world_model.h>
#include <ed/update_request.h>

#include <ros/node_handle.h>
#include <ros/advertise_service_options.h>

//msgs
#include <wire_msgs/WorldEvidence.h>


// ----------------------------------------------------------------------------------------------------

WirePlugin::WirePlugin()
{
}

// ----------------------------------------------------------------------------------------------------

WirePlugin::~WirePlugin()
{
}

// ----------------------------------------------------------------------------------------------------

void WirePlugin::configure(tue::Configuration config)
{
    std::string state_topic;

    config.value("topic", state_topic, tue::REQUIRED);

    ros::NodeHandle nh("~");
//    ros::NodeHandle nh_private;
    ros::SubscribeOptions sub_options = ros::SubscribeOptions::create<wire_msgs::WorldState>(
                    state_topic, 10, boost::bind(&WirePlugin::wireCallback, this, _1), ros::VoidPtr(), &cb_queue_);
    wire_sub_ = nh.subscribe(sub_options);
}

// ----------------------------------------------------------------------------------------------------

void WirePlugin::initialize()
{

}

// ----------------------------------------------------------------------------------------------------

void WirePlugin::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{
    world_model_ = &world;
    update_req_ = &req;
    cb_queue_.callAvailable();
}

// ----------------------------------------------------------------------------------------------------

void WirePlugin::wireCallback(const wire_msgs::WorldStateConstPtr& msg)
{
    ROS_ERROR("[ED WIRE] Callback");
}

ED_REGISTER_PLUGIN(WirePlugin)
