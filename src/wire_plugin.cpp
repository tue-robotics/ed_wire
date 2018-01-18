#include "ed_wire/wire_plugin.h"

#include <ed/entity.h>
#include <ed/world_model.h>
#include <ed/update_request.h>

#include <ros/node_handle.h>
#include <ros/advertise_service_options.h>

#include <geolib/datatypes.h>

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
                    state_topic, 1, boost::bind(&WirePlugin::wireCallback, this, _1), ros::VoidPtr(), &cb_queue_);
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
    // processing all objects from wire
    for(std::vector<wire_msgs::ObjectState>::const_iterator it = msg->objects.begin(); it != msg->objects.end(); ++it)
    {
        std::string ed_id = "wire-"+std::to_string(it->ID);

        const ed::EntityConstPtr e = world_model_->getEntity(ed_id);

        geo::Pose3D pose = geo::Pose3D::identity();

        // Check if the entity is already in the world_model, because could also be a new entity
        if(e)
        {
            if(e->has_pose())
                pose = e->pose();
        }

        bool updated = false;
        for(std::vector<wire_msgs::Property>::const_iterator it2 = it->properties.begin(); it2 != it->properties.end(); ++it2)
        {
            if(it2->attribute == "position")
            {
                if(it2->pdf.dimensions != 3)
                {
                    ROS_ERROR_STREAM("[ED WIRE] Position of ID: '" << it->ID << "' is not of dimension '3'");
                    continue;
                }
                const double& x = it2->pdf.data[0];
                const double& y = it2->pdf.data[1];
                const double& z = it2->pdf.data[2];
                pose.setOrigin(geo::Vec3(x, y, z));
                updated = true;
            }
            else if(it2->attribute == "orientation")
            {
                if(it2->pdf.dimensions != 4)
                {
                    ROS_ERROR_STREAM("[ED WIRE] Orientation of ID: '" << it->ID << "' is not of dimension '4'");
                    continue;
                }
                const double& r = it2->pdf.data[0];
                const double& p = it2->pdf.data[1];
                const double& y = it2->pdf.data[2];
                const double& w = it2->pdf.data[3];
                pose.R.setRotation(geo::Quaternion(r, p, y, w));
                updated = true;
            }
        }

        // Only set pose when position or orientation is updated with a correct measurement.
        // When there is no correct measurement of position OR orientation and entity is not already in the world_model it is not added.
        // TODO: New entities should at least have a correct position, before being allowed to be added to world_model.
        if(updated)
            update_req_->setPose(ed_id, pose);

    }

    // Removing all entities, which originate from wire, which are not in wire anymore. So keeping entities with incorrect measurements
    // TODO: should we keep entities with incorrect measurements?
    for(ed::WorldModel::const_iterator it = world_model_->begin(); it != world_model_->end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;

        // Ignore all entities that are not originating from wire
        if(e->id().str().find("wire") == std::string::npos)
            continue;

        // Check if id is in last wire state
        for(std::vector<wire_msgs::ObjectState>::const_iterator it = msg->objects.begin(); it != msg->objects.end(); ++it)
        {
            std::string ed_id = "wire-"+std::to_string(it->ID);
            if(e->id().str() == ed_id)
               break;

            ROS_DEBUG_STREAM("[ED WIRE] Entity: '" << e->id().str() << "'is not equal to '" << ed_id << "'");
            //Only remove when not equal and all objects in wire_state checked
            if(std::next(it) == msg->objects.end())
            {
                ROS_DEBUG_STREAM("[ED WIRE] Removing entity: '" << e->id().str() << "', because it is not in wire anymore");
                update_req_->removeEntity(e->id());
            }
        }

       // This would also remove entities with incorrect measurements, which should be bad if it only happens
//        if(update_req_->updated_entities.count(e->id()) == 0) // TODO: ?Or should it be != 1?
//        {
//            update_req_->removeEntity(e->id());
//        }
    }
}

ED_REGISTER_PLUGIN(WirePlugin)
