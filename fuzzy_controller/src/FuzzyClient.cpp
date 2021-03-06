#include <ros/ros.h>
#include <tutorial2_controller/FuzzyControlAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/Float64.h"
#include "geometry_msgs/Vector3.h"
#include <string>

#define PI 3.14159265

//class containing the client
class FuzzyClient{

  public:

    FuzzyClient(std::string name, std::string controlName, std::string spName):
	    //Set up the client. It's publishing to topic "fuzzy_control", and is set to auto-spin
      ac(controlName.c_str(), true)
      {
	    //Stores the name
      action_name=std::string(name);

	      //Get connection to a server
	      ROS_INFO("%s Waiting For Server...", action_name.c_str());

	      //Wait for the connection to be valid
	      ac.waitForServer();

	      ROS_INFO("%s Got a Server...", action_name.c_str());
	
        goalsub = n.subscribe(spName.c_str(), 100, &FuzzyClient::GoalCallback, this);
	}


// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state, const tutorial2_controller::FuzzyControlResultConstPtr& result)
{
	ROS_INFO("Finished in state [%s]", state.toString().c_str());

	ROS_INFO("Result: %i", result->ok);
};

// Called once when the goal becomes active
void activeCb()
{
	ROS_INFO("Goal just went active...");
};

// Called every time feedback is received for the goal
void feedbackCb(const tutorial2_controller::FuzzyControlFeedbackConstPtr& feedback)
{
	ROS_INFO("Got Feedback of Progress to Goal: position: %f %f %f", feedback->position.x, feedback->position.y, feedback->position.z);
};

void GoalCallback(const geometry_msgs::Vector3& msg)
{
	  goal.position.x = msg.x;
 	  goal.position.y = msg.y;
	  goal.position.z = msg.z;	
	//Send a goal to the server
	ac.sendGoal(goal, boost::bind(&FuzzyClient::doneCb, this, _1, _2),
		 boost::bind(&FuzzyClient::activeCb, this),
		 boost::bind(&FuzzyClient::feedbackCb, this, _1));
};

private:
	actionlib::SimpleActionClient<tutorial2_controller::FuzzyControlAction> ac;
	std::string action_name;	
	tutorial2_controller::FuzzyControlGoal goal;	
	ros::Subscriber goalsub;
	ros::NodeHandle n;
};

int main (int argc, char **argv)
{
	ros::init(argc, argv, "fuzzy_client");
	
	// create the action client
	// true causes the client to spin its own thread
  	FuzzyClient client(ros::this_node::getName(),"fuzzycontrol1","/vrep_joint");


	ros::spin();
	
	//exit
	return 0;
}
