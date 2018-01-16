#ifndef ED_MOVEIT_PLUGIN_H_
#define ED_MOVEIT_PLUGIN_H_

#include <ed/plugin.h>

#include <ed/types.h>

// Communication
#include <ros/callback_queue.h>
#include <ros/service_server.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

// Configuration
#include <tue/config/configuration.h>

// msgs
#include <wire_msgs/WorldState.h>


class WirePlugin : public ed::Plugin
{

public:

    WirePlugin();

    virtual ~WirePlugin();

    void configure(tue::Configuration config);

    void initialize();

    void process(const ed::WorldModel& world, ed::UpdateRequest& req);

private:

    const ed::WorldModel* world_model_;

    ed::UpdateRequest* update_req_;

    // Communication

    ros::CallbackQueue cb_queue_;

    void wireCallback(const wire_msgs::WorldStateConstPtr& msg);

    ros::Subscriber wire_sub_;

};

#endif
