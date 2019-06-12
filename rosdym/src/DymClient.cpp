#include <ros/ros.h>
#include <rosdym/DymAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/Float64.h"
#include "geometry_msgs/Vector3.h"

#include <vector> //fabio



//class containing the client
class DymClient {

private:
  actionlib::SimpleActionClient<rosdym::DymAction> ac;
  std::string action_name;
  //rosdym::DymGoal goal; fabio
  ros::Subscriber goalsub;
  ros::NodeHandle n;

  /*
  std::vector<rosdym::DymGoal> bufferGoal;
  void SendGoal() {
    if (bufferGoal.size() < 1) return;

    //Send a goal to the server
    ac.sendGoal(bufferGoal[0], boost::bind(&DymClient::doneCb, this, _1, _2),
      boost::bind(&DymClient::activeCb, this),
      boost::bind(&DymClient::feedbackCb, this, _1));

  };
  fabio*/

public:

  DymClient(std::string name, std::string controlName, std::string spName) :
    //Set up the client. It's publishing to topic "fuzzy_control", and is set to auto-spin
    ac(controlName.c_str(), true)
  {
    //Stores the name
    action_name = std::string(name);

    //Get connection to a server
    ROS_INFO("%s Waiting For Server...", action_name.c_str());

    //Wait for the connection to be valid
    ac.waitForServer();

    ROS_INFO("%s Got a Server...", action_name.c_str());

    goalsub = n.subscribe(spName.c_str(), 100, &DymClient::GoalCallback, this);

  }

  // Called once when the goal completes
  void doneCb(const actionlib::SimpleClientGoalState& state, const rosdym::DymResultConstPtr& result)
  {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());

    ROS_INFO("Result: %i", result->ok);


   // if (bufferGoal.size() > 0)
   // {
      //bufferGoal.erase(bufferGoal.begin());
      //if (bufferGoal.size() > 0)
      //  SendGoal();
   // }
  }

  // Called once when the goal becomes active
  void activeCb()
  {
    ROS_INFO("Goal just went active...");
  }

  // Called every time feedback is received for the goal
  void feedbackCb(const rosdym::DymFeedbackConstPtr& feedback)
  {
    ROS_INFO("Got Feedback of Progress to Goal: position: %f \n %f \n %f", feedback->PositionFeedback.x, feedback->PositionFeedback.y, feedback->PositionFeedback.z);
  }

  void GoalCallback(const geometry_msgs::Vector3& msg)
  {
    rosdym::DymGoal goal;//um novo goal...
    goal.PositionGoal.x =msg.x;
    goal.PositionGoal.y =msg.y;
    goal.PositionGoal.z =msg.z;
    ac.sendGoal(goal, boost::bind(&DymClient::doneCb, this, _1, _2),
       boost::bind(&DymClient::activeCb, this),
       boost::bind(&DymClient::feedbackCb, this, _1));
    //bufferGoal.push_back(goal);
    //if (bufferGoal.size() == 1) {//chama o send goal apenas se a fila estivesse vazia, todo o resto � chamado automaticamente pelo cb de completar
    //  SendGoal();
    }
    /*fabio
    //Send a goal to the server
    ac.sendGoal(goal, boost::bind(&DymClient::doneCb, this, _1, _2),
       boost::bind(&DymClient::activeCb, this),
       boost::bind(&DymClient::feedbackCb, this, _1));
    */

 // };



};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dym_client");

  // create the action client
  // true causes the client to spin its own thread
  DymClient client(ros::this_node::getName(), "dymcontrol1", "/left_hand_joint");

  ros::spin();

  //exit
  return 0;
}
