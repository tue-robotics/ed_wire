#include "ed_wire/wire_plugin.h"

#include <ed/entity.h>
#include <ed/world_model.h>
#include <ed/update_request.h>

#include <ros/node_handle.h>
#include <ros/advertise_service_options.h>

#include <geolib/datatypes.h>

//msgs
#include <wire_msgs/WorldEvidence.h>
#include <problib/conversions.h>


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

    config.value("topic", state_topic, tue::config::REQUIRED);

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
    // TODO: What arrtibutes will be provided by wire?
    // TODO: Maybe a more generic implementation based on distribution type and a mapping between wire and ed attributes.
    // TODO: entity doesn't get a type yet.
    // TODO: entity doesn't have a shape yet. Maybe set shape like https://youtu.be/XDc1l_42RDo?t=5m20s


    // processing all objects from wire
    for(const wire_msgs::ObjectState& object_state : msg->objects)
    {
        std::string ed_id = "wire-"+std::to_string(object_state.ID);

        const ed::EntityConstPtr e = world_model_->getEntity(ed_id);

        geo::Pose3D pose = geo::Pose3D::identity();

        for(const wire_msgs::Property& prop : object_state.properties)
        {
            if(prop.attribute == "position")
            {
                // Check if the entity is already in the world_model, because could also be a new entity
                if(e)
                {
                    if(e->has_pose())
                        pose = e->pose();
                }

                // This is probably overkill
                if(prop.pdf.dimensions != 3)
                {
                    ROS_ERROR_STREAM("[ED WIRE] Position of ID: '" << object_state.ID << "' is not of dimension '3'");
                    continue;
                }

                if (prop.pdf.type != problib::PDF::GAUSSIAN)
                {
                    ROS_ERROR_STREAM("[ED WIRE] Position of ID: '" << object_state.ID << "' is not Gaussian");
                    continue;
                }

                const pbl::Gaussian* pos_gauss = pbl::msgToGaussian(prop.pdf);
                const pbl::Vector& pos = pos_gauss->getMean();

                pose.setOrigin(geo::Vec3(pos[0], pos[1], pos[2]));
                update_req_->setPose(ed_id, pose);
            }
            else if(prop.attribute == "class_label")
            {
                const pbl::PDF* pdf = pbl::msgToPDF(prop.pdf);
                std::string label = "";
                pdf->getExpectedValue(label);
                update_req_->setType(ed_id, label);
            }

        }

    }

    // Removing all entities, which originate from wire, which are not in wire anymore. So keeping entities with incorrect measurements
    // TODO: should we keep entities with incorrect measurements?
    for(ed::EntityConstPtr e : world_model_->entities())
    {
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
    }
}

ED_REGISTER_PLUGIN(WirePlugin)
