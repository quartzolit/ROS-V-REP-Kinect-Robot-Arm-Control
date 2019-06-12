#include <ros/ros.h>
 #include <tf/transform_listener.h>
 #include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
// Used API services:
//#include "vrep_common/simRosEnablePublisher.h"
//#include "vrep_common/simRosEnableSubscriber.h"




bool GetTransform( const std::string &frame_name,const std::string &parent_name,
				   tf::TransformListener &ref_listener,
				   ros::Time &ref_now, tf::Quaternion &ref_quat
				   ,geometry_msgs::Vector3 &ref_rotation)
{
tf::StampedTransform transform;//só existe nesse escopo, logo é 'resetada' ao sair da function...
	try
		{
			ref_listener.lookupTransform(frame_name,parent_name,ref_now,transform);

		}
    catch(tf::TransformException &ex) 
		{
			ROS_ERROR("%s",ex.what());
			return false;
		}
	
if(frame_name=="/neck_1" || frame_name=="/left_hand_1")
{
	ref_rotation.x=transform.getOrigin().x();
	ref_rotation.y=transform.getOrigin().y();
	ref_rotation.z=transform.getOrigin().z();
}
	
else
{

	ref_quat= transform.getRotation();

double roll, pitch, yaw;

tf::Matrix3x3(ref_quat).getRPY(roll,pitch,yaw);

	ref_rotation.x = roll;
	ref_rotation.y = pitch;
	ref_rotation.z = yaw;


}
return true;
}

int main(int argc, char** argv) { 

ros::init(argc, argv, "my_skeleton_tf_listener");
ros::NodeHandle node;
ros::Rate rate(50.0); // frequency of operation

ros::Time now = ros::Time(0);
geometry_msgs::Vector3 mPr;

// publisher declaration

ros::Publisher neck_joint = node.advertise<geometry_msgs::Vector3>("neck_joint", 1);
//ros::Publisher head_joint = node.advertise<geometry_msgs::Point>("head_joint", 1);
ros::Publisher torso_joint = node.advertise<geometry_msgs::Vector3>("torso_joint", 1);
ros::Publisher left_shoulder_joint = node.advertise<geometry_msgs::Vector3>("left_shoulder_joint", 1);
ros::Publisher left_elbow_joint = node.advertise<geometry_msgs::Vector3>("left_elbow_joint", 1);
ros::Publisher left_hand_joint = node.advertise<geometry_msgs::Vector3>("left_hand_joint", 1);
ros::Publisher right_shoulder_joint = node.advertise<geometry_msgs::Vector3>("right_shoulder_joint", 1);
ros::Publisher right_elbow_joint = node.advertise<geometry_msgs::Vector3>("right_elbow_joint", 1);
/*ros::Publisher right_hand_joint = node.advertise<geometry_msgs::Quaternion>("right_hand_joint", 1);
ros::Publisher left_hip_joint = node.advertise<geometry_msgs::Point>("left_hip_joint", 1);
ros::Publisher left_knee_joint = node.advertise<geometry_msgs::Point>("left_knee_joint", 1);
ros::Publisher left_foot_joint = node.advertise<geometry_msgs::Point>("left_foot_joint", 1);
ros::Publisher right_hip_joint = node.advertise<geometry_msgs::Point>("right_hip_joint", 1);
ros::Publisher right_knee_joint = node.advertise<geometry_msgs::Point>("right_knee_joint", 1);
ros::Publisher right_foot_joint = node.advertise<geometry_msgs::Point>("right_foot_joint", 1);*/

// listener 
tf::TransformListener listener;
tf::Quaternion quat;


while (node.ok())//main looping
{
//só publica se pegar com sucesso o transform...
   if(GetTransform("/torso_1","/openni_depth_frame",listener,now,quat,mPr)) torso_joint.publish(mPr);
   
   if(GetTransform("/left_shoulder_1","/torso_1",listener,now,quat,mPr)) left_shoulder_joint.publish(mPr);
 
   if(GetTransform("/left_elbow_1","/left_shoulder_1",listener,now,quat,mPr)) left_elbow_joint.publish(mPr);

   if(GetTransform("/right_shoulder_1","/torso_1",listener,now,quat,mPr)) right_shoulder_joint.publish(mPr);

   if(GetTransform("/right_elbow_1","/right_shoulder_1",listener,now,quat,mPr)) right_elbow_joint.publish(mPr);

   if(GetTransform("/neck_1","/openni_depth_frame",listener,now,quat,mPr)) neck_joint.publish(mPr);

   if(GetTransform("/left_hand_1","/left_shoulder_1",listener,now,quat,mPr)) left_hand_joint.publish(mPr);
   

    rate.sleep();
}

return 0;

};
