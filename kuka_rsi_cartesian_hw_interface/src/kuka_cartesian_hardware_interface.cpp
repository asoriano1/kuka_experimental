/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014 Norwegian University of Science and Technology
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Norwegian University of Science and
 *     Technology, nor the names of its contributors may be used to
 *     endorse or promote products derived from this software without
 *     specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Author: Lars Tingelstad <lars.tingelstad@ntnu.no>
 */

#include <kuka_rsi_cartesian_hw_interface/kuka_cartesian_hardware_interface.h>
#include "joint_state_controller/joint_state_controller.h"
#include <robotnik_trajectory_pad/CartesianEuler.h>
#include <stdexcept>
#include <robotnik_msgs/set_odometry.h>
#include <cmath>

namespace kuka_rsi_cartesian_hw_interface
{

KukaHardwareInterface::KukaHardwareInterface() :
    joint_position_(6, 0.0), joint_velocity_(6, 0.0), joint_effort_(6, 0.0), joint_position_command_(6, 0.0), joint_velocity_command_(
        6, 0.0), joint_effort_command_(6, 0.0), joint_names_(6), rsi_initial_joint_positions_(6, 0.0), rsi_joint_position_corrections_(
        6, 0.0), ipoc_(0), n_dof_(6), cartesian_pad_cmds_(6, 0.0), rsi_abs_cart_correction_(6, 0.0)
{
  in_buffer_.resize(1024);
  out_buffer_.resize(1024);
  remote_host_.resize(1024);
  remote_port_.resize(1024);

  if (!nh_.getParam("controller_joint_names", joint_names_))
  {
    ROS_ERROR("Cannot find required parameter 'controller_joint_names' "
      "on the parameter server.");
    throw std::runtime_error("Cannot find required parameter "
      "'controller_joint_names' on the parameter server.");
  }
  // get publishing period
  if (!nh_.getParam("publish_rate", publish_rate_)){
    ROS_ERROR("Parameter 'publish_rate' not set");
    throw std::runtime_error("Cannot find required parameter ");
  }
  
  // realtime publisher
  realtime_pub_.reset(new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(nh_, "joint_states", 4));
  //publishes the cartesian position of the robot
  cart_pos_pub=nh_.advertise<robotnik_msgs::Cartesian_Euler_pose>("cartesian_pos_kuka",10);
  //publishes if the robot is moving through service 
  kuka_moving_pub=nh_.advertise<std_msgs::Bool>("kuka_moving",10);
  
  
  //subscriber
  pad_subs = nh_.subscribe<robotnik_trajectory_pad::CartesianEuler>("/kuka_pad/cartesian_move", 1, &KukaHardwareInterface::padcallback, this);
  //service
  set_kuka_odometry_abs=nh_.advertiseService("setKukaAbs",&KukaHardwareInterface::setKukaOdometry_abs,this);
  set_kuka_odometry_rel=nh_.advertiseService("setKukaRel",&KukaHardwareInterface::setKukaOdometry_rel,this);
  set_kuka_odometry_abs_fast=nh_.advertiseService("setKukaAbsFast",&KukaHardwareInterface::setKukaOdometry_abs_fast,this);
  set_kuka_odometry_rel_fast=nh_.advertiseService("setKukaRelFast",&KukaHardwareInterface::setKukaOdometry_rel_fast,this);

}

KukaHardwareInterface::~KukaHardwareInterface()
{

}
//callback from topic /cartesian_move
void KukaHardwareInterface::padcallback(const robotnik_trajectory_pad::CartesianEuler::ConstPtr& cartesian_move)
{
  cartesian_pad_cmds_[0]=cartesian_move->x;
  cartesian_pad_cmds_[1]=cartesian_move->y;
  cartesian_pad_cmds_[2]=cartesian_move->z;
  cartesian_pad_cmds_[3]=cartesian_move->pitch;
  cartesian_pad_cmds_[4]=cartesian_move->roll;
  cartesian_pad_cmds_[5]=cartesian_move->yaw;
  
}

bool KukaHardwareInterface::read(const ros::Time time, const ros::Duration period)
{
  in_buffer_.resize(1024);

  if (server_->recv(in_buffer_) == 0)
  {
    return false;
  }
 // ROS_INFO("Received from robot:%s", in_buffer_.c_str());
  if (rt_rsi_pub_->trylock()){
    rt_rsi_pub_->msg_.data = in_buffer_;
    rt_rsi_pub_->unlockAndPublish();
  }

  rsi_state_ = RSIState(in_buffer_);
  // limit rate of publishing
  if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0/publish_rate_) < time){
	// try to publish
	if (realtime_pub_->trylock()){
		// we're actually publishing, so increment time
		last_publish_time_ = last_publish_time_ + ros::Duration(1.0/publish_rate_);
		realtime_pub_->msg_.header.stamp = time;
		//ROS_INFO("READ time: %f", (-last_publish_time_.toSec()+time.toSec()));
		//update and publish by /joint_states
		for (std::size_t i = 0; i < n_dof_; ++i)		
		{
			realtime_pub_->msg_.position[i] = DEG2RAD * rsi_state_.positions[i];
							
			realtime_pub_->msg_.velocity[i] = 0;
			
			realtime_pub_->msg_.effort[i] = 0;
			
		}
		//Absolute cartesian pos of the robot
		cart_pos.x=rsi_state_.cart_position[0];
		cart_pos.y=rsi_state_.cart_position[1];
		cart_pos.z=rsi_state_.cart_position[2];
		cart_pos.A=rsi_state_.cart_position[3];
		cart_pos.B=rsi_state_.cart_position[4];
		cart_pos.C=rsi_state_.cart_position[5];
		
		realtime_pub_->unlockAndPublish();
		cart_pos_pub.publish(cart_pos);
	}
  } 
  ipoc_ = rsi_state_.ipoc;

  return true;
}

bool KukaHardwareInterface::write(const ros::Time time, const ros::Duration period)
{
	out_buffer_.resize(1024);
	
  //read from the topic

  /*for (std::size_t i = 0; i < n_dof_; ++i)
  {
    rsi_joint_position_corrections_[i] = (RAD2DEG * joint_position_command_[i]) - rsi_initial_joint_positions_[i];
  }*/

	//ROS_INFO("In Write");
	if((service_set_kuka_abs || service_set_kuka_rel) && n_cyc<=n_cyc_total){
		//write part
		msgs_kuka_moving.data=true;
		n_cyc++;
		float slope=1;
		if(n_cyc<=n_cyc_total*0.25 ){
			//slope=0.75;
			slope=(n_cyc/(0.25*n_cyc_total))*1.333; 
			//ROS_INFO("In slow part Start");
		}else if(n_cyc>=0.75*n_cyc_total){
		//slope=0.75;
			slope=((n_cyc_total-n_cyc)/(0.25*n_cyc_total))*1.333; 
			//ROS_INFO("In slow part END");
		}else{	
			slope=1.333;
			//ROS_INFO("In fast part");
		}
	  
		for (std::size_t i = 0; i < n_dof_; ++i)
		{
		
			rsi_abs_cart_correction_[i]=rsi_abs_cart_correction_[i]+step_tr[i]*slope; 
			rsi_joint_position_corrections_[i]=rsi_abs_cart_correction_[i];
		
		}
 /*
	rsi_abs_cart_correction_[3]=rsi_abs_cart_correction_[3]+step_tr[3]*slope;
	rsi_joint_position_corrections_[3]=rsi_abs_cart_correction_[3];
  
	rsi_abs_cart_correction_[4]=rsi_abs_cart_correction_[4];
	rsi_joint_position_corrections_[4]=rsi_abs_cart_correction_[4];
  
	rsi_abs_cart_correction_[5]=rsi_abs_cart_correction_[5]; 
	rsi_joint_position_corrections_[5]=rsi_abs_cart_correction_[5];
*/
		if(n_cyc>=n_cyc_total-1){
			service_set_kuka_abs=false;
			service_set_kuka_rel=false;
			msgs_kuka_moving.data=false;
		}
	

	
	}else if(!service_set_kuka_abs && !service_set_kuka_rel){
		//ROS_INFO(" NOT In Service");
	
		for (std::size_t i = 0; i < n_dof_-3; ++i)
		{
			//absolute
			rsi_abs_cart_correction_[i]=rsi_abs_cart_correction_[i]+cartesian_pad_cmds_[i];
			rsi_joint_position_corrections_[i]=rsi_abs_cart_correction_[i];
		}
  
		//in kuka [4] is yaw [5] is pitch [6] ir roll
		rsi_abs_cart_correction_[3]=rsi_abs_cart_correction_[3]+cartesian_pad_cmds_[5];
		rsi_joint_position_corrections_[3]=rsi_abs_cart_correction_[3];
	
		rsi_abs_cart_correction_[4]=rsi_abs_cart_correction_[4]+cartesian_pad_cmds_[4];
		rsi_joint_position_corrections_[4]=rsi_abs_cart_correction_[4];
	
		rsi_abs_cart_correction_[5]=rsi_abs_cart_correction_[5]+cartesian_pad_cmds_[3];
		rsi_joint_position_corrections_[5]=rsi_abs_cart_correction_[5];
	
	}

  out_buffer_ = RSICommand('R',rsi_joint_position_corrections_ , ipoc_).xml_doc;
 // ROS_INFO("Send to robot:%s", out_buffer_.c_str());

  server_->send(out_buffer_);
  kuka_moving_pub.publish(msgs_kuka_moving);
  
  return true;
}


//service for abs coord
bool KukaHardwareInterface::setKukaOdometry_abs(robotnik_msgs::set_CartesianEuler_pose::Request &request, robotnik_msgs::set_CartesianEuler_pose::Response &response){	

	
	n_cyc=0;
	
	
	aut_cmds_[0]=request.x-rsi_state_.cart_position[0];
	aut_cmds_[1]=request.y-rsi_state_.cart_position[1];
	aut_cmds_[2]=request.z-rsi_state_.cart_position[2];
	aut_cmds_[3]=request.A-rsi_state_.cart_position[3];
	aut_cmds_[4]=request.B-rsi_state_.cart_position[4];
	if(request.A>0 && rsi_state_.cart_position[5]<0){
		aut_cmds_[5]=-360+request.C-rsi_state_.cart_position[5];
	}else if(request.A<0 && rsi_state_.cart_position[5]>0){
		aut_cmds_[5]=360+request.C-rsi_state_.cart_position[5];
	}else{
	aut_cmds_[5]=request.C-rsi_state_.cart_position[5];
	}  //Because in kuka C angle goes from -179 to 179
	  
	float total_time_tras=sqrt(pow(aut_cmds_[0],2)+pow(aut_cmds_[1],2)+pow(aut_cmds_[2],2))/velocity_trajectory_kuka;
	float total_time_rot=sqrt(pow(aut_cmds_[3],2)+pow(aut_cmds_[4],2)+pow(aut_cmds_[5],2))/velocity_trajectory_kuka_rot;
	
	total_time=fmax(total_time_tras,total_time_rot);
	n_cyc_total=total_time/t_cyc;
	
	 for (std::size_t i = 0; i < n_dof_; ++i)
	{
		if(n_cyc_total>=10){//por si la posicion comandada es la misma en la que está que no divida por cero
			step_tr[i]=aut_cmds_[i]/n_cyc_total;
		}else{
			step_tr[i]=0;
		}
	}
  service_set_kuka_abs=true;
  response.ret=true;
  return true;
}
//service for abs coord
bool KukaHardwareInterface::setKukaOdometry_abs_fast(robotnik_msgs::set_CartesianEuler_pose::Request &request, robotnik_msgs::set_CartesianEuler_pose::Response &response){	
	

	n_cyc=0;
	
	
	aut_cmds_[0]=request.x-rsi_state_.cart_position[0];
	aut_cmds_[1]=request.y-rsi_state_.cart_position[1];
	aut_cmds_[2]=request.z-rsi_state_.cart_position[2]; 
	aut_cmds_[3]=request.A-rsi_state_.cart_position[3];
	aut_cmds_[4]=request.B-rsi_state_.cart_position[4];
	if(request.A>0 && rsi_state_.cart_position[5]<0){
		aut_cmds_[5]=-360+request.C-rsi_state_.cart_position[5];
	}else if(request.A<0 && rsi_state_.cart_position[5]>0){
		aut_cmds_[5]=360+request.C-rsi_state_.cart_position[5];
	}else{
	aut_cmds_[5]=request.C-rsi_state_.cart_position[5];
	}  //Because in kuka  C angle goes from -179 to 179
	  
	float total_time_tras=sqrt(pow(aut_cmds_[0],2)+pow(aut_cmds_[1],2)+pow(aut_cmds_[2],2))/velocity_trajectory_kuka;
	float total_time_rot=sqrt(pow(aut_cmds_[3],2)+pow(aut_cmds_[4],2)+pow(aut_cmds_[5],2))/velocity_trajectory_kuka_rot;
	
	total_time=fmax(total_time_tras,total_time_rot)/velocity_factor;
	n_cyc_total=total_time/t_cyc;
	
	 for (std::size_t i = 0; i < n_dof_; ++i)
	{
		if(n_cyc_total>=10){//por si la posicion comandada es la misma en la que está que no divida por cero
			step_tr[i]=aut_cmds_[i]/n_cyc_total;
		}else{
			step_tr[i]=0;
		}
	}
	service_set_kuka_abs=true;
  response.ret=true;
  return true;
}

//service for rel coord
bool KukaHardwareInterface::setKukaOdometry_rel(robotnik_msgs::set_CartesianEuler_pose::Request &request, robotnik_msgs::set_CartesianEuler_pose::Response &response){	
	
	n_cyc=0;

	aut_cmds_[0]=request.x;
	aut_cmds_[1]=request.y;
	aut_cmds_[2]=request.z;
	aut_cmds_[3]=request.A;
	aut_cmds_[4]=request.B;
	aut_cmds_[5]=request.C;
	/*
	for (std::size_t i = 0; i < n_dof_-3; ++i){
	pos_srv_[i]=rsi_state_.cart_position[i];
	}
	*/
	float total_time_tras=sqrt(pow(aut_cmds_[0],2)+pow(aut_cmds_[1],2)+pow(aut_cmds_[2],2))/velocity_trajectory_kuka;
	float total_time_rot=sqrt(pow(aut_cmds_[3],2)+pow(aut_cmds_[4],2)+pow(aut_cmds_[5],2))/velocity_trajectory_kuka_rot;
	
	total_time=fmax(total_time_tras,total_time_rot);
	
	n_cyc_total=total_time/t_cyc;
	
	 for (std::size_t i = 0; i < n_dof_; ++i)
	{
		if(n_cyc_total>=10){ //por si la posicion comandada es la misma en la que está que no divida por cero
			step_tr[i]=aut_cmds_[i]/n_cyc_total;
		}else{
			step_tr[i]=0;
		}
	}
	service_set_kuka_rel=true;
  response.ret=true;
  return true;
}

//service for rel coord
bool KukaHardwareInterface::setKukaOdometry_rel_fast(robotnik_msgs::set_CartesianEuler_pose::Request &request, robotnik_msgs::set_CartesianEuler_pose::Response &response){	
	
	n_cyc=0;

	aut_cmds_[0]=request.x;
	aut_cmds_[1]=request.y;
	aut_cmds_[2]=request.z;
	aut_cmds_[3]=request.A;
	aut_cmds_[4]=request.B;
	aut_cmds_[5]=request.C;
	
	
	float total_time_tras=sqrt(pow(aut_cmds_[0],2)+pow(aut_cmds_[1],2)+pow(aut_cmds_[2],2))/velocity_trajectory_kuka;
	float total_time_rot=sqrt(pow(aut_cmds_[3],2)+pow(aut_cmds_[4],2)+pow(aut_cmds_[5],2))/velocity_trajectory_kuka_rot;
	
	total_time=fmax(total_time_tras,total_time_rot)/velocity_factor;
	
	n_cyc_total=total_time/t_cyc;
	
	 for (std::size_t i = 0; i < n_dof_; ++i)
	{
		if(n_cyc_total>=10){//por si la posicion comandada es la misma en la que está que no divida por cero
			step_tr[i]=aut_cmds_[i]/n_cyc_total;
		}else{
			step_tr[i]=0;
		}
	}
	service_set_kuka_rel=true;
	response.ret=true;
	return true;
}


void KukaHardwareInterface::start()
{
	//for the service
	service_set_kuka_abs=false; 
	service_set_kuka_rel=false;
	msgs_kuka_moving.data=false;
	
	
	velocity_trajectory_kuka=10; // mm/s 10 example (30 for 12ms) last 5
	velocity_trajectory_kuka_rot=1.5;//grad/s
    t_cyc=0.02; // sec
	velocity_factor=2.5;
  // Wait for connection from robot
  server_.reset(new UDPServer(local_host_, local_port_));

  ROS_INFO_STREAM_NAMED("kuka_hardware_interface", "Waiting for robot!");

  int bytes = server_->recv(in_buffer_);

  // Drop empty <rob> frame with RSI <= 2.3
  if (bytes < 100)
  {
    bytes = server_->recv(in_buffer_);
  }

  rsi_state_ = RSIState(in_buffer_);
  for (std::size_t i = 0; i < n_dof_; ++i)
  {
    joint_position_[i] = DEG2RAD * rsi_state_.positions[i];
    joint_position_command_[i] = joint_position_[i];
    rsi_initial_joint_positions_[i] = rsi_state_.initial_cart_position[i];
  }
  ipoc_ = rsi_state_.ipoc;
  out_buffer_ = RSICommand('R',rsi_initial_joint_positions_, ipoc_).xml_doc;
  server_->send(out_buffer_);
  // Set receive timeout to 1 second
  server_->set_timeout(1000);
  ROS_INFO_STREAM_NAMED("kuka_hardware_interface", "Got connection from robot");
  // initialize time
  last_publish_time_ = ros::Time::now();
  // get joints and allocate message
  for (unsigned i=0; i<n_dof_; i++){
		  //joint_state_.push_back(hw->getHandle(joint_names[i]));
		  realtime_pub_->msg_.name.push_back(joint_names_[i]);
		  realtime_pub_->msg_.position.push_back(0.0);
		  realtime_pub_->msg_.velocity.push_back(0.0);
		  realtime_pub_->msg_.effort.push_back(0.0);
		  
		  
		  
  }

}

void KukaHardwareInterface::configure()
{
  if (nh_.getParam("rsi/listen_address", local_host_) && nh_.getParam("rsi/listen_port", local_port_))
  {
    ROS_INFO_STREAM_NAMED("kuka_hardware_interface",
                          "Setting up RSI server on: (" << local_host_ << ", " << local_port_ << ")");
  }
  else
  {
    ROS_ERROR("Failed to get RSI listen address or listen port from parameter server!");
    throw std::runtime_error("Failed to get RSI listen address or listen port from parameter server.");
  }
  rt_rsi_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::String>(nh_, "rsi_xml_doc", 3));
}




}

