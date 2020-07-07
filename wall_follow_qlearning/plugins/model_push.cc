#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "geometry_msgs/Pose2D.h"
#include <iostream>
#include <cmath>

#ifndef PI
#define PI 3.14159265
#endif

namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
	/// \brief A node use for ROS transport
	private: std::unique_ptr<ros::NodeHandle> rosNode;

	/// \brief A ROS subscriber
	private: ros::Subscriber rosSub;

	/// \brief A ROS callbackqueue that helps process messages
	private: ros::CallbackQueue rosQueue;

	/// \brief A thread the keeps running the rosQueue
	private: std::thread rosQueueThread;	  

    private: float xSpeed = 0, ySpeed = 0, thetaSpeed = 0;
    
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelPush::OnUpdate, this));
        
        // Initialize ros, if it has not already bee initialized.
        if (!ros::isInitialized())
        {
          int argc = 0;
          char **argv = NULL;
          ros::init(argc, argv, "gazebo_client",
              ros::init_options::NoSigintHandler);
        }

        // Create our ROS node. This acts in a similar manner to
        // the Gazebo node
        this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

        // Create a named topic, and subscribe to it.
        ros::SubscribeOptions so =
          ros::SubscribeOptions::create<geometry_msgs::Pose2D>(
              "/" + this->model->GetName() + "/vel_cmd",
              1,
              boost::bind(&ModelPush::OnRosMsg, this, _1),
              ros::VoidPtr(), &this->rosQueue);
        this->rosSub = this->rosNode->subscribe(so);

        // Spin up the queue helper thread.
        this->rosQueueThread =
          std::thread(std::bind(&ModelPush::QueueThread, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      // Apply a small linear velocity to the model.
      float world_angle = this->model->WorldPose().Rot().Yaw();
      float x_setpoint = xSpeed * std::cos(world_angle) + 
                         ySpeed * std::cos(world_angle+PI/2.0);
      float y_setpoint = xSpeed * std::sin(world_angle) + 
                         ySpeed * std::sin(world_angle+PI/2.0);

      float z_setpoint = this->model->RelativeLinearVel().Z();
      this->model->SetLinearVel(ignition::math::Vector3d(x_setpoint, y_setpoint, z_setpoint));
      this->model->SetAngularVel(ignition::math::Vector3d(0, 0, thetaSpeed));
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    
    /// \brief Handle an incoming message from ROS
    /// \param[in] _msg A float value that is used to set the velocity
    /// of the Velodyne.
    public: void OnRosMsg(const geometry_msgs::Pose2DConstPtr &_msg)
    {
        this->xSpeed = _msg->x;
        this->ySpeed = _msg->y;
	this->thetaSpeed = _msg->theta;
    }

    /// \brief ROS helper function that processes messages
    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
