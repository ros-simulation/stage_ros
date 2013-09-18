/*
 *  stageros
 *  Copyright (c) 2008, Willow Garage, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/**

@mainpage

@htmlinclude manifest.html
**/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <signal.h>


// libstage
#include <stage.hh>

// roscpp
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <rosgraph_msgs/Clock.h>

#include "tf/transform_broadcaster.h"

#define USAGE "stageros <worldfile>"
#define IMAGE "image"
#define DEPTH "depth"
#define CAMERA_INFO "camera_info"
#define ODOM "odom"
#define BASE_SCAN "base_scan"
#define BASE_POSE_GROUND_TRUTH "base_pose_ground_truth"
#define CMD_VEL "cmd_vel"

// Our node
class StageNode
{
  private:
    // Messages that we'll send or receive
    sensor_msgs::CameraInfo *cameraMsgs;
    sensor_msgs::Image *imageMsgs;
    sensor_msgs::Image *depthMsgs;
    sensor_msgs::LaserScan *laserMsgs;
    nav_msgs::Odometry *odomMsgs;
    nav_msgs::Odometry *groundTruthMsgs;
    rosgraph_msgs::Clock clockMsg;

    // roscpp-related bookkeeping
    ros::NodeHandle n_;

    // A mutex to lock access to fields that are used in message callbacks
    boost::mutex msg_lock;

    // The models that we're interested in
    std::vector<Stg::ModelCamera *> cameramodels;
    std::vector<Stg::ModelRanger *> lasermodels;
    std::vector<Stg::ModelPosition *> positionmodels;
    std::vector<ros::Publisher> image_pubs_;
    std::vector<ros::Publisher> depth_pubs_;
    std::vector<ros::Publisher> camera_pubs_;
    std::vector<ros::Publisher> laser_pubs_;
    std::vector<ros::Publisher> odom_pubs_;
    std::vector<ros::Publisher> ground_truth_pubs_;
    std::vector<ros::Subscriber> cmdvel_subs_;
    ros::Publisher clock_pub_;
    
    bool isDepthCanonical;

    // A helper function that is executed for each stage model.  We use it
    // to search for models of interest.
    static void ghfunc(Stg::Model* mod, StageNode* node);

    static bool s_update(Stg::World* world, StageNode* node)
    {
      node->WorldCallback();
      // We return false to indicate that we want to be called again (an
      // odd convention, but that's the way that Stage works).
      return false;
    }

    // Appends the given robot ID to the given message name.  If omitRobotID
    // is true, an unaltered copy of the name is returned.
    const char *mapName(const char *name, size_t robotID);

    tf::TransformBroadcaster tf;

    // Last time that we received a velocity command
    ros::Time base_last_cmd;
    ros::Duration base_watchdog_timeout;

    // Current simulation time
    ros::Time sim_time;
    
    // Last time we saved global position (for velocity calculation).
    ros::Time base_last_globalpos_time;
    // Last published global pose of each robot
    std::vector<Stg::Pose> base_last_globalpos;

  public:
    // Constructor; stage itself needs argc/argv.  fname is the .world file
    // that stage should load.
    StageNode(int argc, char** argv, bool gui, const char* fname);
    ~StageNode();

    // Subscribe to models of interest.  Currently, we find and subscribe
    // to the first 'laser' model and the first 'position' model.  Returns
    // 0 on success (both models subscribed), -1 otherwise.
    int SubscribeModels();

    // Our callback
    void WorldCallback();
    
    // Do one update of the world.  May pause if the next update time
    // has not yet arrived.
    bool UpdateWorld();

    // Message callback for a MsgBaseVel message, which set velocities.
    void cmdvelReceived(int idx, const boost::shared_ptr<geometry_msgs::Twist const>& msg);

    // The main simulator object
    Stg::World* world;
};

// since stageros is single-threaded, this is OK. revisit if that changes!
const char *
StageNode::mapName(const char *name, size_t robotID)
{
  if (positionmodels.size() > 1)
  {
    static char buf[100];
    snprintf(buf, sizeof(buf), "/robot_%u/%s", (unsigned int)robotID, name);
    return buf;
  }
  else
    return name;
}

void
StageNode::ghfunc(Stg::Model* mod, StageNode* node)
{
  if (dynamic_cast<Stg::ModelRanger *>(mod))
    node->lasermodels.push_back(dynamic_cast<Stg::ModelRanger *>(mod));
  if (dynamic_cast<Stg::ModelPosition *>(mod))
    node->positionmodels.push_back(dynamic_cast<Stg::ModelPosition *>(mod));
  if (dynamic_cast<Stg::ModelCamera *>(mod))
    node->cameramodels.push_back(dynamic_cast<Stg::ModelCamera *>(mod));
}

void
StageNode::cmdvelReceived(int idx, const boost::shared_ptr<geometry_msgs::Twist const>& msg)
{
  boost::mutex::scoped_lock lock(msg_lock);
  this->positionmodels[idx]->SetSpeed(msg->linear.x, 
                                      msg->linear.y, 
                                      msg->angular.z);
  this->base_last_cmd = this->sim_time;
}

StageNode::StageNode(int argc, char** argv, bool gui, const char* fname)
{
  this->sim_time.fromSec(0.0);
  this->base_last_cmd.fromSec(0.0);
  double t;
  ros::NodeHandle localn("~");
  if(!localn.getParam("base_watchdog_timeout", t))
    t = 0.2;
  this->base_watchdog_timeout.fromSec(t);
  
  if(!localn.getParam("is_depth_canonical", isDepthCanonical))
    isDepthCanonical = true;
  

  // We'll check the existence of the world file, because libstage doesn't
  // expose its failure to open it.  Could go further with checks (e.g., is
  // it readable by this user).
  struct stat s;
  if(stat(fname, &s) != 0)
  {
    ROS_FATAL("The world file %s does not exist.", fname);
    ROS_BREAK();
  }

  // initialize libstage
  Stg::Init( &argc, &argv );

  if(gui)
    this->world = new Stg::WorldGui(600, 400, "Stage (ROS)");
  else
    this->world = new Stg::World();

  // Apparently an Update is needed before the Load to avoid crashes on
  // startup on some systems.
  // As of Stage 4.1.1, this update call causes a hang on start.
  //this->UpdateWorld();
  this->world->Load(fname);

  // We add our callback here, after the Update, so avoid our callback
  // being invoked before we're ready.
  this->world->AddUpdateCallback((Stg::world_callback_t)s_update, this);

  this->world->ForEachDescendant((Stg::model_callback_t)ghfunc, this);
  if (lasermodels.size() > 0 && lasermodels.size() != positionmodels.size())
  {
    ROS_FATAL("number of position models and laser models must be equal in "
              "the world file.");
    ROS_BREAK();
  }
  else if (cameramodels.size() > 0 && cameramodels.size() != positionmodels.size())
  {
    ROS_FATAL("number of position models and camera models must be equal in "
              "the world file.");
    ROS_BREAK();
  }
  size_t numRobots = positionmodels.size();
  ROS_INFO("found %u position and laser(%u)/camera(%u) pair%s in the file", 
           (unsigned int)numRobots, (unsigned int) lasermodels.size(), (unsigned int) cameramodels.size(), (numRobots==1) ? "" : "s");

  this->laserMsgs = new sensor_msgs::LaserScan[numRobots];
  this->odomMsgs = new nav_msgs::Odometry[numRobots];
  this->groundTruthMsgs = new nav_msgs::Odometry[numRobots];
  this->imageMsgs = new sensor_msgs::Image[numRobots];
  this->depthMsgs = new sensor_msgs::Image[numRobots];
  this->cameraMsgs = new sensor_msgs::CameraInfo[numRobots];
}


// Subscribe to models of interest.  Currently, we find and subscribe
// to the first 'laser' model and the first 'position' model.  Returns
// 0 on success (both models subscribed), -1 otherwise.
//
// Eventually, we should provide a general way to map stage models onto ROS
// topics, similar to Player .cfg files.
int
StageNode::SubscribeModels()
{
  n_.setParam("/use_sim_time", true);

  for (size_t r = 0; r < this->positionmodels.size(); r++)
  {
    if(this->lasermodels.size()>r && this->lasermodels[r])
    {
      this->lasermodels[r]->Subscribe();
    }
    else if (this->lasermodels.size()>0)
    {
      ROS_ERROR("no laser");
      return(-1);
    }
    if(this->positionmodels[r])
    {
      this->positionmodels[r]->Subscribe();
    }
    else
    {
      ROS_ERROR("no position");
      return(-1);
    }
    if(this->cameramodels.size()>r && this->cameramodels[r])
    {
      this->cameramodels[r]->Subscribe();
    }
    else if (this->cameramodels.size()>0)
    {
      ROS_ERROR_STREAM("no camera " << this->cameramodels.size());
      return(-1);
    }
    if (this->lasermodels.size()>r)
      laser_pubs_.push_back(n_.advertise<sensor_msgs::LaserScan>(mapName(BASE_SCAN,r), 10));
    odom_pubs_.push_back(n_.advertise<nav_msgs::Odometry>(mapName(ODOM,r), 10));
    ground_truth_pubs_.push_back(n_.advertise<nav_msgs::Odometry>(mapName(BASE_POSE_GROUND_TRUTH,r), 10));
    if (this->cameramodels.size()>r){
      image_pubs_.push_back(n_.advertise<sensor_msgs::Image>(mapName(IMAGE,r), 10));
      depth_pubs_.push_back(n_.advertise<sensor_msgs::Image>(mapName(DEPTH,r), 10));
      camera_pubs_.push_back(n_.advertise<sensor_msgs::CameraInfo>(mapName(CAMERA_INFO,r), 10));
    }
    cmdvel_subs_.push_back(n_.subscribe<geometry_msgs::Twist>(mapName(CMD_VEL,r), 10, boost::bind(&StageNode::cmdvelReceived, this, r, _1)));
  }
  clock_pub_ = n_.advertise<rosgraph_msgs::Clock>("/clock",10);
  return(0);
}

StageNode::~StageNode()
{
  delete[] laserMsgs;
  delete[] odomMsgs;
  delete[] groundTruthMsgs;
  delete[] imageMsgs;
  delete[] depthMsgs;
  delete[] cameraMsgs;
}

bool
StageNode::UpdateWorld()
{
  return this->world->UpdateAll();
}

void
StageNode::WorldCallback()
{
  boost::mutex::scoped_lock lock(msg_lock);

  this->sim_time.fromSec(world->SimTimeNow() / 1e6);
  // We're not allowed to publish clock==0, because it used as a special
  // value in parts of ROS, #4027.
  if(this->sim_time.sec == 0 && this->sim_time.nsec == 0)
  {
    ROS_DEBUG("Skipping initial simulation step, to avoid publishing clock==0");
    return;
  }

  // TODO make this only affect one robot if necessary
  if((this->base_watchdog_timeout.toSec() > 0.0) &&
      ((this->sim_time - this->base_last_cmd) >= this->base_watchdog_timeout))
  {
    for (size_t r = 0; r < this->positionmodels.size(); r++)
      this->positionmodels[r]->SetSpeed(0.0, 0.0, 0.0);
  }

  // Get latest laser data
  for (size_t r = 0; r < this->lasermodels.size(); r++)
		{
			const std::vector<Stg::ModelRanger::Sensor>& sensors = this->lasermodels[r]->GetSensors();
		
		if( sensors.size() > 1 )
			ROS_WARN( "ROS Stage currently supports rangers with 1 sensor only." );

		// for now we access only the zeroth sensor of the ranger - good
		// enough for most laser models that have a single beam origin
		const Stg::ModelRanger::Sensor& s = sensors[0];
		
    if( s.ranges.size() )
			{
      // Translate into ROS message format and publish
      this->laserMsgs[r].angle_min = -s.fov/2.0;
      this->laserMsgs[r].angle_max = +s.fov/2.0;
      this->laserMsgs[r].angle_increment = s.fov/(double)(s.sample_count-1);
      this->laserMsgs[r].range_min = s.range.min;
      this->laserMsgs[r].range_max = s.range.max;
      this->laserMsgs[r].ranges.resize(s.ranges.size());
      this->laserMsgs[r].intensities.resize(s.intensities.size());
			
      for(unsigned int i=0; i<s.ranges.size(); i++)
				{
					this->laserMsgs[r].ranges[i] = s.ranges[i];
					this->laserMsgs[r].intensities[i] = (uint8_t)s.intensities[i];
				}
			
      this->laserMsgs[r].header.frame_id = mapName("base_laser_link", r);
      this->laserMsgs[r].header.stamp = sim_time;
      this->laser_pubs_[r].publish(this->laserMsgs[r]);
			}

    // Also publish the base->base_laser_link Tx.  This could eventually move
    // into being retrieved from the param server as a static Tx.
    Stg::Pose lp = this->lasermodels[r]->GetPose();
    tf::Quaternion laserQ;
    laserQ.setRPY(0.0, 0.0, lp.a);
    tf::Transform txLaser =  tf::Transform(laserQ,
                                            tf::Point(lp.x, lp.y, this->positionmodels[r]->GetGeom().size.z+lp.z));
    tf.sendTransform(tf::StampedTransform(txLaser, sim_time,
                                          mapName("base_link", r),
                                          mapName("base_laser_link", r)));
    }
    
    for (size_t r = 0; r < this->positionmodels.size(); r++)
		{
    // Send the identity transform between base_footprint and base_link
    tf::Transform txIdentity(tf::createIdentityQuaternion(),
                             tf::Point(0, 0, 0));
    tf.sendTransform(tf::StampedTransform(txIdentity,
                                          sim_time,
                                          mapName("base_footprint", r),
                                          mapName("base_link", r)));

    // Get latest odometry data
    // Translate into ROS message format and publish
    this->odomMsgs[r].pose.pose.position.x = this->positionmodels[r]->est_pose.x;
    this->odomMsgs[r].pose.pose.position.y = this->positionmodels[r]->est_pose.y;
    this->odomMsgs[r].pose.pose.orientation = tf::createQuaternionMsgFromYaw(this->positionmodels[r]->est_pose.a);
    Stg::Velocity v = this->positionmodels[r]->GetVelocity();
    this->odomMsgs[r].twist.twist.linear.x = v.x;
    this->odomMsgs[r].twist.twist.linear.y = v.y;
    this->odomMsgs[r].twist.twist.angular.z = v.a;

    //@todo Publish stall on a separate topic when one becomes available
    //this->odomMsgs[r].stall = this->positionmodels[r]->Stall();
    //
    this->odomMsgs[r].header.frame_id = mapName("odom", r);
    this->odomMsgs[r].header.stamp = sim_time;

    this->odom_pubs_[r].publish(this->odomMsgs[r]);

    // broadcast odometry transform
    tf::Quaternion odomQ;
    tf::quaternionMsgToTF(odomMsgs[r].pose.pose.orientation, odomQ);
    tf::Transform txOdom(odomQ, 
                         tf::Point(odomMsgs[r].pose.pose.position.x,
                                   odomMsgs[r].pose.pose.position.y, 0.0));
    tf.sendTransform(tf::StampedTransform(txOdom, sim_time,
                                          mapName("odom", r),
                                          mapName("base_footprint", r)));

    // Also publish the ground truth pose and velocity
    Stg::Pose gpose = this->positionmodels[r]->GetGlobalPose();
    tf::Quaternion q_gpose;
    q_gpose.setRPY(0.0, 0.0, gpose.a);
    tf::Transform gt(q_gpose, tf::Point(gpose.x, gpose.y, 0.0));
    // Velocity is 0 by default and will be set only if there is previous pose and time delta>0
    Stg::Velocity gvel(0,0,0,0);
    if (this->base_last_globalpos.size()>r){
      Stg::Pose prevpose = this->base_last_globalpos.at(r);
      double dT = (this->sim_time-this->base_last_globalpos_time).toSec();
      if (dT>0)
        gvel = Stg::Velocity(
          (gpose.x - prevpose.x)/dT, 
          (gpose.y - prevpose.y)/dT, 
          (gpose.z - prevpose.z)/dT, 
          Stg::normalize(gpose.a - prevpose.a)/dT
        );
      this->base_last_globalpos.at(r) = gpose;
    }else //There are no previous readings, adding current pose...
      this->base_last_globalpos.push_back(gpose);

    this->groundTruthMsgs[r].pose.pose.position.x     = gt.getOrigin().x();
    this->groundTruthMsgs[r].pose.pose.position.y     = gt.getOrigin().y();
    this->groundTruthMsgs[r].pose.pose.position.z     = gt.getOrigin().z();
    this->groundTruthMsgs[r].pose.pose.orientation.x  = gt.getRotation().x();
    this->groundTruthMsgs[r].pose.pose.orientation.y  = gt.getRotation().y();
    this->groundTruthMsgs[r].pose.pose.orientation.z  = gt.getRotation().z();
    this->groundTruthMsgs[r].pose.pose.orientation.w  = gt.getRotation().w();
    this->groundTruthMsgs[r].twist.twist.linear.x = gvel.x;
    this->groundTruthMsgs[r].twist.twist.linear.y = gvel.y;
    this->groundTruthMsgs[r].twist.twist.linear.z = gvel.z;
    this->groundTruthMsgs[r].twist.twist.angular.z = gvel.a;

    this->groundTruthMsgs[r].header.frame_id = mapName("odom", r);
    this->groundTruthMsgs[r].header.stamp = sim_time;

    this->ground_truth_pubs_[r].publish(this->groundTruthMsgs[r]);
  }
  
  this->base_last_globalpos_time = this->sim_time;
  
  for (size_t r = 0; r < this->cameramodels.size(); r++)
  {
    // Get latest image data
    // Translate into ROS message format and publish
    if (this->image_pubs_[r].getNumSubscribers()>0 && this->cameramodels[r]->FrameColor()) {
       this->imageMsgs[r].height=this->cameramodels[r]->getHeight();
       this->imageMsgs[r].width=this->cameramodels[r]->getWidth();
       this->imageMsgs[r].encoding="rgba8";
       //this->imageMsgs[r].is_bigendian="";
       this->imageMsgs[r].step=this->imageMsgs[r].width*4;
       this->imageMsgs[r].data.resize(this->imageMsgs[r].width*this->imageMsgs[r].height*4);

       memcpy(&(this->imageMsgs[r].data[0]),this->cameramodels[r]->FrameColor(),this->imageMsgs[r].width*this->imageMsgs[r].height*4);

       //invert the opengl weirdness
       int height = this->imageMsgs[r].height - 1;
       int linewidth = this->imageMsgs[r].width*4;

       char* temp = new char[linewidth];
       for (int y = 0; y < (height+1)/2; y++) 
       {
            memcpy(temp,&this->imageMsgs[r].data[y*linewidth],linewidth);
            memcpy(&(this->imageMsgs[r].data[y*linewidth]),&(this->imageMsgs[r].data[(height-y)*linewidth]),linewidth);
            memcpy(&(this->imageMsgs[r].data[(height-y)*linewidth]),temp,linewidth);
       }

        this->imageMsgs[r].header.frame_id = mapName("camera", r);
        this->imageMsgs[r].header.stamp = sim_time;

        this->image_pubs_[r].publish(this->imageMsgs[r]);      
    }
    //Get latest depth data
    //Translate into ROS message format and publish
    //Skip if there are no subscribers
    if (this->depth_pubs_[r].getNumSubscribers()>0 && this->cameramodels[r]->FrameDepth()) {
      this->depthMsgs[r].height=this->cameramodels[r]->getHeight();
      this->depthMsgs[r].width=this->cameramodels[r]->getWidth();
      this->depthMsgs[r].encoding=this->isDepthCanonical?sensor_msgs::image_encodings::TYPE_32FC1:sensor_msgs::image_encodings::TYPE_16UC1;
      //this->depthMsgs[r].is_bigendian="";
      int sz = this->isDepthCanonical?sizeof(float):sizeof(uint16_t);
      size_t len = this->depthMsgs[r].width*this->depthMsgs[r].height;
      this->depthMsgs[r].step=this->depthMsgs[r].width*sz;
      this->depthMsgs[r].data.resize(len*sz);

      //processing data according to REP118
      if (this->isDepthCanonical){
        double nearClip = this->cameramodels[r]->getCamera().nearClip();
        double farClip = this->cameramodels[r]->getCamera().farClip();
        memcpy(&(this->depthMsgs[r].data[0]),this->cameramodels[r]->FrameDepth(),len*sz);
        float * data = (float*)&(this->depthMsgs[r].data[0]);
        for (size_t i=0;i<len;++i)
          if(data[i]<=nearClip) 
            data[i] = -INFINITY;
          else if(data[i]>=farClip) 
            data[i] = INFINITY;
      }
      else{
        int nearClip = (int)(this->cameramodels[r]->getCamera().nearClip() * 1000);
        int farClip = (int)(this->cameramodels[r]->getCamera().farClip() * 1000);
        for (size_t i=0;i<len;++i){
          int v = (int)(this->cameramodels[r]->FrameDepth()[i]*1000);
          if (v<=nearClip || v>=farClip) v = 0;
          ((uint16_t*)&(this->depthMsgs[r].data[0]))[i] = (uint16_t) ((v<=nearClip || v>=farClip) ? 0 : v );
        }
      }
      
      //invert the opengl weirdness
      int height = this->depthMsgs[r].height - 1;
      int linewidth = this->depthMsgs[r].width*sz;

      char* temp = new char[linewidth];
      for (int y = 0; y < (height+1)/2; y++) 
      {
           memcpy(temp,&this->depthMsgs[r].data[y*linewidth],linewidth);
           memcpy(&(this->depthMsgs[r].data[y*linewidth]),&(this->depthMsgs[r].data[(height-y)*linewidth]),linewidth);
           memcpy(&(this->depthMsgs[r].data[(height-y)*linewidth]),temp,linewidth);
      }

      this->depthMsgs[r].header.frame_id = mapName("camera", r);
      this->depthMsgs[r].header.stamp = sim_time;
      this->depth_pubs_[r].publish(this->depthMsgs[r]);
    }
    
    //sending camera's tf and info only if image or depth topics are subscribed to
    if ((this->image_pubs_[r].getNumSubscribers()>0 && this->cameramodels[r]->FrameColor())
    || (this->depth_pubs_[r].getNumSubscribers()>0 && this->cameramodels[r]->FrameDepth()))
    {
       
      Stg::Pose lp = this->cameramodels[r]->GetPose();
      tf::Quaternion Q; Q.setRPY(
        (this->cameramodels[r]->getCamera().pitch()*M_PI/180.0)-M_PI, 
        0.0, 
        lp.a+(this->cameramodels[r]->getCamera().yaw()*M_PI/180.0)-this->positionmodels[r]->GetPose().a
        );
        
      tf::Transform tr =  tf::Transform(Q, tf::Point(lp.x, lp.y, this->positionmodels[r]->GetGeom().size.z+lp.z));
      tf.sendTransform(tf::StampedTransform(tr, sim_time,
                                          mapName("base_link", r),
                                          mapName("camera", r)));
      
      this->cameraMsgs[r].header.frame_id = mapName("camera", r);
      this->cameraMsgs[r].header.stamp = sim_time;
      this->cameraMsgs[r].height = this->cameramodels[r]->getHeight();
      this->cameraMsgs[r].width = this->cameramodels[r]->getWidth();
      
      double fx,fy,cx,cy;
      cx = this->cameraMsgs[r].width / 2.0;
      cy = this->cameraMsgs[r].height / 2.0;
      double fovh = this->cameramodels[r]->getCamera().horizFov()*M_PI/180.0;
      double fovv = this->cameramodels[r]->getCamera().vertFov()*M_PI/180.0;
      //double fx_ = 1.43266615300557*this->cameramodels[r]->getWidth()/tan(fovh);
      //double fy_ = 1.43266615300557*this->cameramodels[r]->getHeight()/tan(fovv);
      fx = this->cameramodels[r]->getWidth()/(2*tan(fovh/2));
      fy = this->cameramodels[r]->getHeight()/(2*tan(fovv/2));
      
      //ROS_INFO("fx=%.4f,%.4f; fy=%.4f,%.4f", fx, fx_, fy, fy_);
 
      
      this->cameraMsgs[r].D.resize(4, 0.0);

      this->cameraMsgs[r].K[0] = fx;
      this->cameraMsgs[r].K[2] = cx;
      this->cameraMsgs[r].K[4] = fy;
      this->cameraMsgs[r].K[5] = cy;
      this->cameraMsgs[r].K[8] = 1.0;

      this->cameraMsgs[r].R[0] = 1.0;
      this->cameraMsgs[r].R[4] = 1.0;
      this->cameraMsgs[r].R[8] = 1.0;

      this->cameraMsgs[r].P[0] = fx;
      this->cameraMsgs[r].P[2] = cx;
      this->cameraMsgs[r].P[5] = fy;
      this->cameraMsgs[r].P[6] = cy;
      this->cameraMsgs[r].P[10] = 1.0;
      
      this->camera_pubs_[r].publish(this->cameraMsgs[r]);                                    

    }   
     
    
  }

  this->clockMsg.clock = sim_time;
  this->clock_pub_.publish(this->clockMsg);
}

int 
main(int argc, char** argv)
{ 
  if( argc < 2 )
  {
    puts(USAGE);
    exit(-1);
  }

  ros::init(argc, argv, "stageros");

  bool gui = true;
  for(int i=0;i<(argc-1);i++)
  {
    if(!strcmp(argv[i], "-g"))
      gui = false;
  }

  StageNode sn(argc-1,argv,gui,argv[argc-1]);

  if(sn.SubscribeModels() != 0)
    exit(-1);

  boost::thread t = boost::thread(boost::bind(&ros::spin));

  // New in Stage 4.1.1: must Start() the world.
  sn.world->Start();

  // TODO: get rid of this fixed-duration sleep, using some Stage builtin
  // PauseUntilNextUpdate() functionality.
  ros::WallRate r(10.0);
  while(ros::ok() && !sn.world->TestQuit())
  {
    if(gui)
      Fl::wait(r.expectedCycleTime().toSec());
    else
    {
      sn.UpdateWorld();
      r.sleep();
    }
  }
  t.join();

  exit(0);
}

