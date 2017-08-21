#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/vs/vpServo.h>
#include <visp3/robot/vpSimulatorCamera.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpProjectionDisplay.h>
#include <visp3/vs/vpServoDisplay.h>
#include <visp3/robot/vpWireFrameSimulator.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include "std_msgs/String.h"
#include <iostream>
#include <cstdlib>
#include <fstream>
#include "wam_msgs/MatrixMN.h"
#include "wam_srvs/JointMove.h"
#include "sensor_msgs/JointState.h"
//#include <conio.h>

#define DofNum 2
using namespace std;

vpMatrix fJe(6,DofNum) ;
vpFeaturePoint p;  //current point
vpFeaturePoint pd; //desired point
bool drawOnBoard=true;

 std::vector<double> stopPose;
 
 //scan along columns
void wamToolJacobianCallback(const wam_msgs::MatrixMN::ConstPtr& jacobianMessage)
{
    for (int i = 0; i < 6; i++)
	{
// 	 for (int j = 0; j < 7; j++)
// 		{
          fJe[i][0]=jacobianMessage->data[i+0*6];
	  fJe[i][1]=jacobianMessage->data[i+3*6];

		//} 
	}
// 	cout<<"jacobian Read:"<<endl;//<<fJe<<endl;
// 	cout<<fJe[0][0]<<"  "<<fJe[0][1]<<endl;
// 	cout<<fJe[1][0]<<"  "<<fJe[1][1]<<endl;
// 	cout<<fJe[2][0]<<"  "<<fJe[2][1]<<endl;
// 	cout<<fJe[3][0]<<"  "<<fJe[3][1]<<endl;
// 	cout<<fJe[4][0]<<"  "<<fJe[4][1]<<endl;
// 	cout<<fJe[5][0]<<"  "<<fJe[5][1]<<endl;
}
 std::vector<double> initial_Joint_pose;
std::vector<double> position_integration(std::vector<double> previousAngles,vpColVector current_vel, double t_delta )
{   
   std::vector<double> nowAngles=initial_Joint_pose;
   
   //cout<<"start doing integration!"<<endl;
//     for (int i=0; i<7;i++)
//     {
//       switch(i)
//       {
// 	case 0:
// 	   nowAngles.push_back(previousAngles[i]+current_vel[0]*t_delta);
// 	   break;
// 	case 3:
// 	   nowAngles.push_back(previousAngles[i]+current_vel[3]*t_delta);
// 	   break;
// 	default:
// 	  nowAngles.push_back(previousAngles[i]);
// 	  break;
//       }
//        
//     }
    nowAngles.at(0)=previousAngles[0]+current_vel[0]*t_delta;
    nowAngles.at(1)=1.57;
    nowAngles.at(2)=1.57;
     nowAngles.at(3)=previousAngles[3]+ current_vel[1]*t_delta;
    nowAngles.at(4)=-4.71;
    nowAngles.at(5)=-1.57;
    nowAngles.at(6)=0;
    
    return nowAngles;
}

bool currentPoseReadDone=false;
void currentPointCallback(const geometry_msgs::Point::ConstPtr& msg)
 {
    p.set_x(msg->x);
    p.set_y(msg->y);
    
    currentPoseReadDone=true;
   // cout<<"current received!:"<<p.get_x()<<" "<<p.get_y()<<endl;
 }
 
 void desiredPointCallback(const geometry_msgs::Point::ConstPtr& msg)
 {
    pd.set_x(msg->x);
    pd.set_y(msg->y);
    if(msg->z==0)
      drawOnBoard=false;
    else
      drawOnBoard=true;
   // cout<<"desired received!:"<<pd.get_x()<<" "<<pd.get_y()<<endl;
 }
 

 bool initialReadDone=false;
 void wamPoseCallback(const sensor_msgs::JointState::ConstPtr& msg)
 {
   //if(!initialReadDone)
   {
    initial_Joint_pose=msg->position;
    
    initialReadDone=true;
    //cout<<"joint pose obtained:"<<endl;//<<initial_Joint_pose[1]<<endl;
     //cout<<initial_Joint_pose[0]<<"  "<<initial_Joint_pose[1]<<"  "<<initial_Joint_pose[2]<<" "<<initial_Joint_pose[3]<<" "<< initial_Joint_pose[4]
    // <<" "<<initial_Joint_pose[5]<<" "<<initial_Joint_pose[6]<<endl;
   }
 }
 
 std::vector<float> convert2Float(std::vector<double> v)
 {
   std::vector<float> t;
   for(int i=0;i<v.size();i++)
     t.push_back((float)v[i]);
   return t;
}

void printOutStdVector(std::vector<double> v)
{
  cout<<"print out v:"<<endl;
  for(int i=0;i<v.size();i++)
  {
    cout<<v.at(i)<<"  ";
  }
  cout<<endl;
}

int main(int argc, char **argv )
{
    //define stop pose
   stopPose.push_back(0.143);
   stopPose.push_back(1.57);
   stopPose.push_back(1.57);
   stopPose.push_back(1.64);
   stopPose.push_back(-4.71);
   stopPose.push_back(-1.57);
   stopPose.push_back(0);
   //cout<<"stopPose  ";
   //printOutStdVector(stopPose);
   
    //------------------------------------------------------------------
    cout<<("define the task") <<endl;
    cout<<("\t we want an eye-in-hand control law")<<endl ;
    cout<<("\t robot is controlled in the camera frame") <<endl;
   task.setServo(vpServo::EYETOHAND_L_cVf_fJe) ;
   task.setInteractionMatrixType(vpServo::CURRENT) ;
  
   cout<<("Display task information " )<<endl ;
   
   
   
     double convergence_threshold = 0.0025; //025 ;
      double error =1000 ;
    unsigned int iter=0 ;
    
    vpHomogeneousMatrix cMf ;//camera's position wrt robot base reference frame 4*4
//     cMf[0][0] = -0.07149830271 ;  cMf[0][1] = -0.409727361;  cMf[0][2] = 0.9094017167;  cMf[0][3] = -0.3320618852;
//     cMf[1][0] = -0.9931621713 ;  cMf[1][1] = 0.07203075161;  cMf[1][2] = -0.09187204276;  cMf[1][3] = 0.4548481991;
//     cMf[2][0] = -0.05703951667 ;  cMf[2][1] = -0.8979247162;  cMf[2][2] = -0.4364375071;  cMf[2][3] = 0.3257839661;
//     cMf[3][0] = 0 ;  cMf[3][1] = 0;  cMf[3][2] = 0;  cMf[3][3] = 1;
    
      cMf[0][0] = 1 ;  cMf[0][1] = 0;  cMf[0][2] = 0;  cMf[0][3] = 0.41;
    cMf[1][0] = 0 ;  cMf[1][1] = -1;  cMf[1][2] = 0;  cMf[1][3] = 0.37;
    cMf[2][0] =0 ;  cMf[2][1] = 0;  cMf[2][2] = -1;  cMf[2][3] = 0.642;
    cMf[3][0] = 0 ;  cMf[3][1] = 0;  cMf[3][2] = 0;  cMf[3][3] = 1;
    
    task.set_cVf(cMf);
   // cout<<"cMf Set"<<endl;
    double lambda_av =0.2;
    task.setLambda(lambda_av);
    // cout<<"task define finished"<<endl ;
     
      int it = 0 ;
   
    double alpha = 1 ; //1 ;
    double beta =3 ; //3 ;
    std::cout << "alpha 0.7" << std::endl;
   // std::cin >> alpha ;
    std::cout << "beta 5" << std::endl;
   // std::cin >> beta ;
     int iteratorCount=0;
    int MaxIteratorCount=100;
    
     
    
    
    
     
  	//initialize the ROS system and become a node.
	ros::init(argc, argv, "ibvstest2_node");
	ros::NodeHandle nh;
	ros::Subscriber subP = nh.subscribe("/chris_tracker/currentPoint", 1000, currentPointCallback);
	ros::Subscriber subPd = nh.subscribe("/chris_tracker/desiredPoint", 1000, desiredPointCallback);
        ros::Publisher pubTask = nh.advertise<std_msgs::String>("/chris_tracker/taskFinished", 1);
	ros::Subscriber jacobian_sub = nh.subscribe("zeus/wam/jacobian",1,wamToolJacobianCallback);
	ros::Subscriber wam_pos_sub=nh.subscribe("/zeus/wam/joint_states",1,wamPoseCallback);
	ros::ServiceClient Joint_move_client = nh.serviceClient<wam_srvs::JointMove>("/zeus/wam/joint_move");
	
	//command the robot to its initial position
        wam_srvs::JointMove mv_srv;
      mv_srv.request.joints = convert2Float( stopPose);
      cout<<"send the robot to its initial position, request created::   "<<endl;
      cout<<"send to robot to initial  position ";
      printOutStdVector(stopPose);
       cout << "Press any key to continue..." << endl;
       getchar();
      Joint_move_client.call(mv_srv);
      cout<<"service called!, robot to initial position, "<<endl;
     cout << "Press any key to continue the servoing loop..." << endl;
       getchar();
       
	//initialize p and Pd
	p.set_x(100);
	p.set_y(100);
	pd.set_x(0);
	pd.set_y(0);
	
// 	//test for publishing
	ros::Rate loop_rate(2);
// 	while (ros::ok())
// 	{
// 	std_msgs::String msg;
// 	std::stringstream ss;
// 	ss << "t" ;
// 	msg.data = ss.str();
// 	pubTask.publish(msg);
// 	
// 	 wam_srvs::JointMove mv_srv;
// 	std::vector<float> jointAngles;
// 	mv_srv.request.joints = jointAngles;
// 	Joint_move_client.call(mv_srv);
// 	
// 	
//           ros::spinOnce();
// 	  loop_rate.sleep();
// 	}
	
 
     task.print() ;
     
     bool adonce=false;
      adonce=true;
      task.addFeature(p,pd) ;
	//  cout<<"add featurePoint:"<<endl;
	  double t1=0;
	  std::vector<double> previousJointState=stopPose;
	  bool initialRunMark=false;
	  
	  bool taskinProgress=true;
    while(ros::ok())
    {
      ros::spinOnce();
      if(!initialRunMark)
        previousJointState=initial_Joint_pose;
     // loop_rate.sleep(); 
      //cout<<"inside while!"<<endl;
      if(currentPoseReadDone&&initialReadDone)
      {
	//cout<<"initialReadDone!"<<endl;
	if(!adonce)
	{
	 
	}
       try {
    double t2=t1;
    //get robot's initial pose
     
    if(!taskinProgress)
    {
       vpServo task ;
       task.setServo(vpServo::EYETOHAND_L_cVf_fJe) ;
   task.setInteractionMatrixType(vpServo::CURRENT) ;
   task.set_cVf(cMf);
    task.setLambda(lambda_av);
     task.addFeature(p,pd) ;
        cout<<"-----------new task created!"<<"---------press enter to continue-------------"<<endl;
	getchar();
    }
    
    
    //std::vector<double> stopJointPose=
    
    vpColVector dotq_prev (DofNum); // this is to hold the previous velocities to integrate
   
    cout<<"error:: "<<error<<"     threshold:: "<<convergence_threshold<<endl;
    
     //  cout<<"feature List number::  "<<task.featureList.size()<<endl;
     //  cout<<"desired feature List number::  "<<task.desiredFeatureList.size()<<endl;
       if(task.featureList.size()==0)
	 task.addFeature(p,pd) ;
      //  cout<<"now feature List number::  "<<task.featureList.size()<<endl;
     //  cout<<"now desired feature List number::  "<<task.desiredFeatureList.size()<<endl;
       t1=vpTime::measureTimeMs();
      if(initialRunMark)
	error = ( task.getError() ).sumSquare() ;
    if(error > convergence_threshold)
    {
      initialRunMark=true;
 //   cout<<"t1: "<<t1<<endl;
    
     // cout<<"enter while loop:: "<<iteratorCount<<endl;
//       if(iteratorCount>=MaxIteratorCount)
// 	continue;
      std::cout << "---------------------------------------------" << iter++ <<std::endl ;
      vpColVector dotq(DofNum);
      //robot.get_eJe(fJe) ;
      //get fJe from topic jacobian/////////////////////////////////////////////////////////
      //vpMatrix fJe(6,7);
      //define fJe;
      //fJe[0][0]=0;
      task.set_fJe(fJe) ; //in while loop, always update fJe
    //  cout<<"fJe set done!"<<endl;
      //we also need to update the current point position in the image.////use it in the ros topic subscriber.
      //pd already updated!
       //task.print() ;
      // Compute the adaptative gain (speed up the convergence)
      double gain ;
      if (iter>2)
      {
        if (std::fabs(alpha) <= std::numeric_limits<double>::epsilon())
          gain = lambda_av ;
        else
        {
          gain = alpha * exp (-beta * ( task.getError() ).sumSquare() ) +  lambda_av;
        }
      }
      else gain = lambda_av ;
//       if (SAVE==1)
//         gain = gain/5 ;
     // vpTRACE("%f %f %f %f  %f",alpha, beta, lambda_av, ( task.getError() ).sumSquare(),  gain) ;
      task.setLambda(gain) ;
    //   cout<<"gain set to task!"<<endl;
      
      dotq = task.computeControlLaw() ;
      cout<<"dotq computed!: "<<dotq<<endl;
      cout<<"previous pose  ";
       printOutStdVector(previousJointState);
      
      //
      // robot.setVelocity(vpRobot::ARTICULAR_FRAME, v) ; //set the joint velocity to the robot
      //using ros service to set the joint velocity to the robot.///////////////////////////////////////////////////////////////
      t2 = vpTime::measureTimeMs();
     // cout<<"t2::   "<<t2<<endl;
      
      double deltaT=t2-t1;
    //  cout<<"previous Joint Angles: "<<previousJointState[0]<<endl;
   //   cout<<"deltaT::   "<<deltaT<<endl;
      //set the joint angles to the robot
      std::vector<double> currentJointState= position_integration(previousJointState,dotq,0.15);
       cout<<"current pose, will be sent to the robot ";
       printOutStdVector(currentJointState);
       
    //  cout<<"integrated joint angles::   "<<endl;
      previousJointState=currentJointState;
      
   
      iteratorCount++;
      
      
      // cout<<"current updated!:"<<p.get_x()<<" "<<p.get_y()<<endl;
      //  cout<<"desired updated!:"<<pd.get_x()<<" "<<pd.get_y()<<endl;
       error = ( task.getError() ).sumSquare() ;
      cout<<"new error:: "<<error<<endl;
      std::cout << "|| s - s* || = "<< error<<std::endl ;
      
         wam_srvs::JointMove mv_srv;
      mv_srv.request.joints = convert2Float( currentJointState);
       cout<<"request created:: ";
       
       cout << "Press any key to continue..." << endl;
    //   getchar();
      Joint_move_client.call(mv_srv);
       cout<<"service called! robot moved!"<<endl;
       
     
       
//       if (error>7)
//         {
//           cout<<("Error detected while tracking visual features") <<endl;
//          /////////////////////
// 	  //using ros service to set the robot to stop.////////////////////////////////////////////////////////////////////////
// 	  //set the joint angles to the robot, reset the robot to its initial pose
//       wam_srvs::JointMove mv_srv;
//       mv_srv.request.joints = convert2Float(initial_Joint_pose);
//       //Joint_move_client.call(mv_srv);
//       cout<<"called service to stop the robot "<<endl;
//           exit(1) ;
//         }
        
       t1 = vpTime::measureTimeMs();
      
       
    }
    else //task finished move to the next task.
    {
      task.kill();
      cout<<"-----------task killed!"<<"---------move to next task-------------"<<endl;
      taskinProgress=false;
      	std_msgs::String msg;
	std::stringstream ss;
	ss << "t" ;
	msg.data = ss.str();
	pubTask.publish(msg);
    }
    loop_rate.sleep();
    //robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;///////////////////////////////////////////////////////using ros to stop the robot motion
    
 }
  catch(vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
    }
    }
    
 
    task.kill();
}

