#include <ros/ros.h>
#include <rosdym/DymAction.h>
#include <actionlib/server/simple_action_server.h>
#include<armadillo>
#include<math.h>
#include "rosdym/RobotInfo.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Vector3.h"
#include <vector>


//Class for containing the server
class DymServer {
public:

  DymServer(std::string name, std::string controlName, std::string InfoName) :
    as(n, controlName, boost::bind(&DymServer::executeCB, this, _1), false)
  {

    action_name = std::string(name);

    as.registerPreemptCallback(boost::bind(&DymServer::preemptCB, this));

    //Set initial values to global variables
    Initialize();
    //Start the server
    as.start();

    //Subscribers
    subinfo = n.subscribe(InfoName.c_str(), 1, &DymServer::SensorCallBack, this);

    //Publishers
    goalpub = n.advertise<geometry_msgs::Vector3>("/output_track_hand", 1);
    pubinfo = n2.advertise<rosdym::RobotInfo>("/output_robot_info", 1);

}


//Callback for handling preemption. Reset your helpers here.
//Note that you still have to check for preemption in your work method to break it off
void preemptCB()
{
  ROS_INFO("%s got preempted!", action_name.c_str());
  result.ok = 0;
  as.setPreempted(result, "I got Preempted!");
}

//Callback for processing a goal
void executeCB(const rosdym::DymGoalConstPtr& goal)
{

  //If the server has been killed, don't process
  if (!as.isActive() || as.isPreemptRequested()) return;

  //Run the processDymFeedbacking at 100Hz
  ros::Rate rate(1000);


  //Setup some local variables
  bool success = true;

  //int count=0;

  while(1)
  {


    targetxyz(0)=goal->PositionGoal.x;
    targetxyz(1)=goal->PositionGoal.y;
    targetxyz(2)=goal->PositionGoal.z;

    goalpub.publish(goal->PositionGoal);

    dist_restante=sqrt(pow(targetxyz[0]-xyz[0],2)+ pow(targetxyz[1]-xyz[1],2)+pow(targetxyz[2]-xyz[2],2));
    dist_base=sqrt(pow(targetxyz[0],2)+ pow(targetxyz[1],2)+pow(targetxyz[2],2));

    if (dist_restante < oldist)
         oldist = dist_restante;
    else{
      minimum = oldist;
    }

    if(dist_base>d_base_max)
    {
      dist_restante=d_base_max;
    }

    if(minimum < 99999 &&  dist_restante/dist_base <= 0.1)
    {
      result.ok = 1;
      as.setSucceeded(result);
      break;
    }
    joint_target_velocities(0)=10;//*dist_restante/dist_base;
    joint_target_velocities(1)=10;//*dist_restante/dist_base;
    joint_target_velocities(2)=10;//*dist_restante/dist_base;

    lock_.lock();
    q[0]=m_info.jointpos1;
    q[1]=m_info.jointpos2;
    q[2]=m_info.jointpos3;

    dq(0)=m_info.jointvel1;
    dq(1)=m_info.jointvel1;
    dq(2)=m_info.jointvel1;

    torque(0)=m_info.jointfor1;
    torque(1)=m_info.jointfor2;
    torque(2)=m_info.jointfor3;
    lock_.unlock();


    xyzz[0]=L[0]*cos(q[0])+L[1]*cos(q[0])*cos(q[1])+L[2]*cos(q[0])*cos(q[1])*cos(q[2])-L[2]*cos(q[0])*sin(q[1])*sin(q[2]);
    xyzz[1]=L[0]*cos(q[0])+L[1]*cos(q[1])*sin(q[0])+L[2]*sin(q[0])*cos(q[1])*cos(q[2])-L[2]*sin(q[0])*sin(q[1])*sin(q[2]);
    xyzz[2]=L[1]*sin(q[1])+L[2]*(cos(q[1])*sin(q[2])+cos(q[2])*sin(q[1]))+.14;

    for (int i=0;i<3;i++)
    {
      xyz(i)=xyzz[i];
    }

    jee[0][2] = -L[2] * sin(q[1]+q[2])*cos(q[0]);
    jee[1][2] = -L[2] * sin(q[1]+q[2])*sin(q[0]);
    jee[2][2] = L[2]*cos(q[1]+q[2]);
    jee[0][1] = -cos(q[0])*(L[2]*sin(q[1]+q[2])+L[1]*sin(q[1]));
    jee[1][1] = -sin(q[0])*(L[2]*sin(q[1]+q[2])+L[1]*sin(q[1]));
    jee[2][1] = L[2] * cos(q[1]+q[2]) + L[1]*cos(q[1]);
    jee[0][0] = -sin(q[0])*(L[0]+jee[2][1]);
    jee[1][0] =cos(q[0])*(L[0]+jee[2][1]);

    for (int i=0;i<3;i++)
    {
      for (int j=0;j<3;j++)
      {
        JEE(i,j)=jee[i][j];

     }
    }

    jcom1[0][0] = L[0] * -sin(q[0])/2;
    jcom1[1][0] = L[0] * cos(q[0])/2;
    jcom1[5][0] = 1.0;


    jcom2[0][1] = L[1] * (cos(q[1]) * sin(q[0]) - cos(q[0]) * sin(q[1]))/2;
    jcom2[1][1] = L[1] * (-cos(q[1]) * cos(q[0]) - sin(q[1]) * sin(q[0]))/2;
    jcom2[4][1] = 1.0;
    jcom2[0][0] = L[1]*(cos(q[0])*sin(q[1])-cos(q[1])*sin(q[0]))/2-L[0]*sin(q[0]);
    jcom2[1][0] = L[1] * (sin(q[0])*sin(q[1]) + cos(q[0])*cos(q[1]))/2 + 2*jcom1[1][0];
    jcom2[4][0] = 1.0;


    jcom3[0][2] = L[2]*(cos(q[2])*sin(q[0]) - cos(q[0])*cos(q[1])*sin(q[2]))/2;
    jcom3[1][2] = L[2]*(-cos(q[2])*cos(q[0]) - cos(q[1])*sin(q[0])*sin(q[2]))/2;
    jcom3[2][2] = -L[2]*sin(q[1])*sin(q[2])/2;
    jcom3[4][2] = 1.0;
    jcom3[0][1] = L[2] * -cos(q[0])*cos(q[2])*sin(q[1])/2 - L[1]*cos(q[0])*sin(q[1]);
    jcom3[1][1] = L[2] * -cos(q[2])*sin(q[0])*sin(q[1])/2 - L[1]*sin(q[0])*sin(q[1]);
    jcom3[2][1] = (L[2]*cos(q[1])*cos(q[2])/2)+L[1]*cos(q[1]);
    jcom3[4][1] = 1.0;
    jcom3[0][0] = L[2]*(cos(q[0])*sin(q[2]) - cos(q[1])*cos(q[2])*sin(q[0]))/2 -L[1]*cos(q[1])*sin(q[0])-L[0]*sin(q[0]);
    jcom3[1][0] = L[2]*(sin(q[0])*sin(q[2]) + cos(q[0])*cos(q[1])*cos(q[2]))/2 + L[1]*cos(q[0])*cos(q[1])+2*jcom1[1][0];
    jcom3[4][0] = 1.0;

    for (int i=0;i<6;i++)
    {
      for (int j=0;j<3;j++)
      {
        JCOM1(i,j)=jcom1[i][j];
        JCOM2(i,j)=jcom2[i][j];
        JCOM3(i,j)=jcom3[i][j];
      }
    }


    M11[0][0]=m1;
    M11[1][1]=m1;
    M11[2][2]=m1;
    M11[3][3]=0.001667; //prototipo de robo M11[4][4]=0.001667;rosdym2 0.001005;
    M11[4][4]=0.001667;//prototipo de robo M11[5][5]=0.001667;rosdym2 0.001005;
    M11[5][5]=0.001667;//prototipo de robo M11[6][6]=0.001667;rosdym2 0.0009983;

    M22[0][0]=m2;
    M22[1][1]=m2;
    M22[2][2]=m2;
    M22[3][3]=0.008121;//prototipo de robo M22[4][4]=0.008121;rosdym2 0.002832;
    M22[4][4]=0.008121;//prototipo de robo M22[5][5]=0.008121;rosdym2 0.002556;
    M22[5][5]=0.001242;//prototipo de robo M22[6][6]=0.001242;rosdym2 0.001645;

    M33[0][0]=m3;
    M33[1][1]=m3;
    M33[2][2]=m3;
    M33[3][3]=0.0005561; //prototipo de robo M33[4][4]=0.0005561; rosdym2 0.005671;
    M33[4][4]=0.0005561;//prototipo de robo M33[5][5]=0.0005561; rosdym2 0.005111;
    M33[5][5]=0.0001747; //prototipo de robo M33[6][6]=0.0001747; rosdym2 0.001645;


    for (int i=0;i<6;i++)
    {
      for (int j=0;j<6;j++)
      {
        if (i==j)
        {
          M1(i,j)=M11[i][j];
          M2(i,j)=M22[i][j];
          M3(i,j)=M33[i][j];
        }
      }
    }

    Mq=JCOM1.t() * M1 *JCOM1 + JCOM2.t() * M2 *JCOM2 + JCOM3.t() * M3 *JCOM3;

    for (int j=0;j<6;j++)
    {
      g(j)=gravity[j];
    }

    Mq_g= JCOM1.t() * M1 *g + JCOM2.t() * M2 *g + JCOM3.t() * M3 *g;

    Mx_inv = JEE * inv(Mq) * JEE.t();

    svd(Mu,ms,Mv,Mx_inv);

    for (int i=0;i < ms.n_elem;i++)
    {
      if(ms(i)<1e-5)
      {
        ms(i)=0;
      }
      else
        ms(i)= 1/ms.at(i);
    }

    Mx= Mv.t()*diagmat(ms)*Mu.t();

    float kp=100;
    float kv=2;

    u_xyz= Mx*(kp*(targetxyz-xyz));

    u=JEE.t()*u_xyz - Mq*(kv*dq) - Mq_g;

  //  u=u*-1;

    for(int i=0;i<3;i++)
    {
       if (torque.at(i) * u.at(i) <0)
      {
        joint_target_velocities(i)=joint_target_velocities.at(i)*-1;
      }
    }

    robot_info.jointpos1=0;
    robot_info.jointpos2=0;
    robot_info.jointpos3=0;

    robot_info.jointfor1=abs(u.at(0));
    robot_info.jointfor2=abs(u.at(1));
    robot_info.jointfor3=abs(u.at(2));

    robot_info.jointvel1=joint_target_velocities.at(0);
    robot_info.jointvel2=joint_target_velocities.at(1);
    robot_info.jointvel3=joint_target_velocities.at(2);

    pubinfo.publish(robot_info);


    feedback.PositionFeedback.x = xyz[0];
    feedback.PositionFeedback.y = xyz[1];
    feedback.PositionFeedback.z = xyz[2];

    //Publish feedback to action client
    as.publishFeedback(feedback);

    //Check for ROS kill
    if (!ros::ok())
    {
      success = false;
      ROS_INFO("%s Shutting Down", action_name.c_str());
      break;

    }

    //If the server has been killed/preempted, stop processing
    if (!as.isActive() || as.isPreemptRequested()) return;

    //Sleep for rate time

    rate.sleep();
    ros::spinOnce();
//     count++;
  }

  //Publish the result if the goal wasn't preempted
  if (success)
  {
    result.ok = 1;
    as.setSucceeded(result);
  }
  else
  {
    result.ok = 0;
    as.setAborted(result, "I Failed!");
   }


}

void Initialize()
{
  targetxyz.zeros(3);
  xyz.zeros(3);
  dq.zeros(3);
  g.zeros(6);//(6,fill::zeros);
  joint_target_velocities.zeros(3);//(3,fill::zeros);
  u.zeros(3);
  u_xyz.zeros(3);//(3,fill::zeros);
  torque.zeros(3);//(3,fill::zeros);
  ms.zeros(3);

  JEE.zeros(3,3);//(3,3,fill::zeros);
  JCOM1.zeros(6,3);//(6,3,fill::zeros);
  JCOM2.zeros(6,3);//(6,3,fill::zeros);
  JCOM3.zeros(6,3);//(6,3,fill::zeros);
  M1.zeros(6,6);//(6,6,fill::zeros);
  M2.zeros(6,6);//(6,6,fill::zeros);
  M3.zeros(6,6);//(6,6,fill::zeros);
  Mq.zeros(6,6);//(3,3,fill::zeros);
  Mq_g.zeros(3,3);//(3,3,fill::zeros);
  Mx_inv.zeros(3,3);
  Mx_inv.zeros(3,3);
  Mx_inv.zeros(3,3);
  Mx_inv.zeros(3,3);
  Mx_inv.zeros(3,3);
  Mx.zeros(3,3);
  Mu.zeros(3,3);
  Mv.zeros(3,3);//(3,3,fill::zeros);

  L[0]=0.000;//269;//prototipo de robo L[0]=0.35; rosdym2 0.000269;
  L[1]=0.42;//prototipo de robo L[1]=0.42; rosdym2 0.053212;
  L[2]=0.225;//prototipo de robo L[2]=0.225; rosdym2 0.053148;
  dist_base=1;

  for(int i=0;i<3;i++)
  {
    for(int j=0;j<3;j++)
    {
      jee[i][j]=0;
    }
  }

  for(int i=0;i<6;i++)
  {
    for(int j=0;j<3;j++)
    {
      jcom1[i][j]=0;
      jcom2[i][j]=0;
      jcom3[i][j]=0;
    }
  }
  for(int i=0;i<6;i++)
  {
    gravity[i]=0;
    for(int j=0;j<6;j++)
    {
      M11[i][j]=0;
      M22[i][j]=0;
      M33[i][j]=0;
    }
  }

  gravity[2]=-9.81;
  m1=0.15;//prototipo de robo m1=0.15; rosdym2 1.002;
  m2=1.171;//prototipo de robo m2=1.171 rosdym2 0.05;
  m3=0.329;//prototipo de robo m3=0.329; rosdym2 0.1;
/*
  sensorpos[0]=0;
  sensorpos[1]=0;
  sensorpos[2]=0;

  sensorvel[0]=0;
  sensorvel[1]=0;
  sensorvel[2]=0;

  sensorfor[0]=0;
  sensorfor[1]=0;
  sensorfor[2]=0;
  */
  q[0]=0;
  q[1]=0;
  q[2]=0;
  minimum=999999;
  oldist=minimum;
  d_base_max=0.985;


}

void SensorCallBack(const rosdym::RobotInfo& msg)
{

  memcpy(&m_info, &msg, sizeof(rosdym::RobotInfo));
}


protected:
  ros::NodeHandle n;
  ros::NodeHandle n2;

  //Variables Subscriber
  ros::Subscriber subinfo;

  //Variables Publisher
  ros::Publisher pubinfo;

  ros::Publisher goalpub;

  rosdym::RobotInfo robot_info;
  rosdym::RobotInfo m_info;

  //Variables Actionlib
  actionlib::SimpleActionServer<rosdym::DymAction> as;
  rosdym::DymFeedback feedback;
  rosdym::DymResult result;
  std::string action_name;
  boost::recursive_mutex lock_;

  //Variables control

  float q[3], L[3], minimum,oldist,xyzz[3], dist_restante,dist_base;
  float jee[3][3], jcom1[6][3], jcom2[6][3], jcom3[6][3];
  float m1, m2, m3, M11[6][6], M22[6][6], M33[6][6];
  float gravity[6],d_base_max;


  arma::fvec targetxyz;//(3,fill::zeros);
  arma::fvec xyz;//(3,fill::zeros);
  arma::fvec dq;//(3,fill::zeros);
  arma::fvec g;//(6,fill::zeros);
  arma::fvec joint_target_velocities;//(3,fill::zeros);
  arma::fvec u;//(3,fill::zeros);
  arma::fvec torque;//(3,fill::zeros);
  arma::fvec u_xyz;

  arma::fmat JEE;//(3,3,fill::zeros);
  arma::fmat JCOM1;//(6,3,fill::zeros);
  arma::fmat JCOM2;//(6,3,fill::zeros);
  arma::fmat JCOM3;//(6,3,fill::zeros);
  arma::fmat M1;//(6,6,fill::zeros);
  arma::fmat M2;//(6,6,fill::zeros);
  arma::fmat M3;//(6,6,fill::zeros);
  arma::fmat Mq;//(3,3,fill::zeros);
  arma::fmat Mq_g;//(3,3,fill::zeros);
  arma::fmat Mx_inv;//(3,3,fill::zeros);
  arma::fmat Mx;
  arma::fmat Mu;
  arma::fmat Mv;
  arma::fvec ms;
};

//Used by ROS to actually create the node. Could theoretically spawn more than one server
int main(int argc, char** argv)
{
  ros::init(argc, argv, "dym_server");

  //Just a check to make sure the usage was correct
  if (argc != 1)
  {
    ROS_INFO("Usage: dym_server");
    return 1;
  }

  //Spawn the server
  DymServer server(ros::this_node::getName(), "dymcontrol1", "/vrep_joint_info");

  ros::spin();

  return 0;
}
