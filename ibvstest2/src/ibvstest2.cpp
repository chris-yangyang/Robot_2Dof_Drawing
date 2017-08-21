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
#include <math.h>
#define DofNum 2
using namespace std;

vpMatrix fJe(6,DofNum) ;
vpFeaturePoint p;  //current point
vpFeaturePoint pd; //desired point
bool drawOnBoard=true;

 std::vector<double> stopPose;
 std::vector<double> getColorPose;
 std::vector<double> leaveColorPoolPose;
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
std::vector<double> position_integration(std::vector<double> previousAngles,vpColVector current_vel, double t_delta , double zz)
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
    
    if(zz!=0)  //drawing on board
      nowAngles.at(1)=1.57;
    else            //slightly uplift the arm, drawing not on board
      nowAngles.at(1)=1.53;
    
    nowAngles.at(2)=1.57;
     nowAngles.at(3)=previousAngles[3]+ current_vel[1]*t_delta;
    nowAngles.at(4)=-4.71;
    nowAngles.at(5)=-1.57;
    nowAngles.at(6)=0;
    
    return nowAngles;
}
 bool initialRunMark=false;
bool currentPoseReadDone=false;
  int taskCount=0;
void currentPointCallback(const geometry_msgs::Point::ConstPtr& msg)
 {
    p.set_x(msg->x);
    p.set_y(msg->y);
    
    currentPoseReadDone=true;
   // cout<<"current received!:"<<p.get_x()<<" "<<p.get_y()<<endl;
 }
 double pdz=-1;
 void desiredPointCallback(const geometry_msgs::Point::ConstPtr& msg)
 {
    pd.set_x(msg->x);
    pd.set_y(msg->y);
    pdz=msg->z;
    if(msg->z==0)
     drawOnBoard=false;
    else
      drawOnBoard=true;
   if( !initialRunMark)
     pdz=0;
   if(taskCount<2)
     pdz=0;
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

double computeError()
{
  double deltaX=p.get_x()-pd.get_x();
  double detalY=p.get_y()-pd.get_y();
  return sqrt(deltaX*deltaX+detalY*detalY);
}

//"[0.53196,1.39524,1.09715,0.955755,-4.0458,-1.292023,-0.000904]"
//"[0.1037,1.167,1.3004,1.27976,-4.7181,-1.54276,-0.0039043]"

int main(int argc, char **argv )
{
    //define stop pose
   stopPose.push_back(0.143);
   stopPose.push_back(1.53);
   stopPose.push_back(1.57);
   stopPose.push_back(1.6898);
   stopPose.push_back(-4.71);
   stopPose.push_back(-1.57);
   stopPose.push_back(0);
   //cout<<"stopPose  ";
   
  getColorPose.push_back(0.53196);
  getColorPose.push_back(1.40524);
  getColorPose.push_back(1.09715);
    getColorPose.push_back(0.955755);
      getColorPose.push_back(-4.0458);
	getColorPose.push_back(-1.292023);
	  getColorPose.push_back(-0.000904);
	  
leaveColorPoolPose.push_back(0.41348);
leaveColorPoolPose.push_back(1.13189);
  leaveColorPoolPose.push_back(1.2167);
    leaveColorPoolPose.push_back(0.98064);
      leaveColorPoolPose.push_back(-4.31997);
	leaveColorPoolPose.push_back(-1.391);
	  leaveColorPoolPose.push_back(-0.00586);
   //printOutStdVector(stopPose);
    //------------------------------------------------------------------
    double convergence_threshold = 0.01; //025 ;
    double error =1000 ;
    unsigned int iter=0 ;
    vpHomogeneousMatrix cMf ;//camera's position wrt robot base reference frame 4*4
//     cMf[0][0] = -0.07149830271 ;  cMf[0][1] = -0.409727361;  cMf[0][2] = 0.9094017167;  cMf[0][3] = -0.3320618852;
//     cMf[1][0] = -0.9931621713 ;  cMf[1][1] = 0.07203075161;  cMf[1][2] = -0.09187204276;  cMf[1][3] = 0.4548481991;
//     cMf[2][0] = -0.05703951667 ;  cMf[2][1] = -0.8979247162;  cMf[2][2] = -0.4364375071;  cMf[2][3] = 0.3257839661;
//     cMf[3][0] = 0 ;  cMf[3][1] = 0;  cMf[3][2] = 0;  cMf[3][3] = 1; 

//     cMf[0][0] = 1 ;  cMf[0][1] = 0;  cMf[0][2] = 0;  cMf[0][3] = 0.41;
//     cMf[1][0] = 0 ;  cMf[1][1] = -1;  cMf[1][2] = 0;  cMf[1][3] = 0.37;
//     cMf[2][0] =0 ;  cMf[2][1] = 0;  cMf[2][2] = -1;  cMf[2][3] = 0.642;
//     cMf[3][0] = 0 ;  cMf[3][1] = 0;  cMf[3][2] = 0;  cMf[3][3] = 1;

    cMf[0][0] = 1 ;  cMf[0][1] = 0;  cMf[0][2] = 0;  cMf[0][3] = 0.55824;
    cMf[1][0] = 0 ;  cMf[1][1] = -1;  cMf[1][2] = 0;  cMf[1][3] =0.37;
    cMf[2][0] =0 ;  cMf[2][1] = 0;  cMf[2][2] = -1;  cMf[2][3] = 0.7642728;
    cMf[3][0] = 0 ;  cMf[3][1] = 0;  cMf[3][2] = 0;  cMf[3][3] = 1;
    double lambda_av =0.4;
    int it = 0 ;
    double alpha = 1 ; //1 ;
    double beta =3 ; //3 ;
    int iteratorCount=0;
    int MaxIteratorCount=300;
    
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
      Joint_move_client.call(mv_srv);///////////////////////////////////////////////////
      cout<<"service called!, robot to initial position, "<<endl;
     cout << "Press any key to continue the servoing loop..." << endl;
      getchar();
      ros::Rate loop_rate(3);//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	  double t1=0;
	  std::vector<double> previousJointState=stopPose;
	 
	  
	  bool taskinProgress=true;

	  vpServo task ;
	
    while(ros::ok())
    {
      ros::spinOnce();
//             wam_srvs::JointMove mv_srv3;
//       mv_srv3.request.joints = convert2Float( initial_Joint_pose);
//        cout << "Dig into:-->>" << endl;
//        //getchar();
//       Joint_move_client.call(mv_srv3);
//        loop_rate.sleep();
      if(!initialRunMark)
        previousJointState=initial_Joint_pose;
     // loop_rate.sleep(); 
      //cout<<"inside while!"<<endl;
      if(currentPoseReadDone&&initialReadDone)
      {
       try {
    double t2=t1; 
    //if(!taskinProgress)
    {
//        if(initialRunMark)
//       {
// 	error = ( task.getError() ).sumSquare() ;
//        cout<<"using task.getError, error:: "<<error<<"     threshold:: "<<convergence_threshold<<endl;
//       }
       if(!taskinProgress||!initialRunMark)
       {
	
	 taskinProgress=true;
       task.setServo(vpServo::EYETOHAND_L_cVf_fJe) ;
       task.setInteractionMatrixType(vpServo::MEAN) ;
       task.set_cVf(cMf);
       task.setLambda(lambda_av);
        if( !initialRunMark)
          pdz=0;
        if(task.featureList.size()==0)
	 task.addFeature(p,pd) ;
	ros::spinOnce();
	 if( !initialRunMark)
           pdz=0;
	  if(taskCount<2)
            pdz=0;
       task.print() ;
      
        cout<<"-----------new task created!"<<"---------press enter to continue-------------"<<endl;
	//getchar();
	 error=100;
	 iter=0;
	 taskCount++;
       }
	 vpColVector dotq_prev (DofNum); // this is to hold the previous velocities to integrate
   
          cout<<"error:: "<<error<<"     threshold:: "<<convergence_threshold<<endl;
  
       if(task.featureList.size()==0)
	 task.addFeature(p,pd) ;
       t1=vpTime::measureTimeMs();
     
      cout<<"error:: "<<error<<"     threshold:: "<<convergence_threshold<<endl;
    if(error > convergence_threshold&&iter<MaxIteratorCount)
    {
      initialRunMark=true;
      std::cout << "---------------------------------------------" << iter++ <<std::endl ;
      vpColVector dotq(DofNum);
      task.set_fJe(fJe) ; //in while loop, always update fJe
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
      task.setLambda(gain) ;
      dotq = task.computeControlLaw() ;
      cout<<"dotq computed!: "<<dotq<<endl;
      cout<<"previous pose  ";
       printOutStdVector(previousJointState);
      t2 = vpTime::measureTimeMs();
      double deltaT=t2-t1;
       if( !initialRunMark)
          pdz=0;
       if(taskCount<2)
           pdz=0;
      std::vector<double> currentJointState= position_integration(previousJointState,dotq,0.080,pdz);
       cout<<"current pose, will be sent to the robot ";
       printOutStdVector(currentJointState);
      previousJointState=currentJointState;
      iteratorCount++;
      //error = ( task.getError() ).sumSquare() ;
      error= computeError();
      cout<<"new error:: "<<error<<endl;
      std::cout << "|| s - s* || = "<< error<<std::endl ;
         wam_srvs::JointMove mv_srv;
      mv_srv.request.joints = convert2Float( currentJointState);
       cout<<"request created:: ";
       //cout << "Press any key to continue..." << endl;
    //   getchar();
      Joint_move_client.call(mv_srv);
       cout<<"service called! robot moved!"<<endl;
      if (error>7)
     {
      cout<<("Error detected while tracking visual features") <<endl;
      wam_srvs::JointMove mv_srv;
      mv_srv.request.joints = convert2Float(stopPose);
      Joint_move_client.call(mv_srv);
      cout<<"called service to stop the robot "<<endl;
      exit(1) ;
     }
       cout<<"task count::"<<taskCount<<endl; 
     t1 = vpTime::measureTimeMs();
    }
    else //task finished move to the next task.
    {
      //test if need to change color, check current z value ==-2;
      //store previous state
      //if needed
      if(taskCount>=100)
      {
	//move to levve color pose
	//move to get color pose
	//
	wam_srvs::JointMove mv_srv2;
      mv_srv2.request.joints = convert2Float( leaveColorPoolPose);
       cout<<"request created:: ";
       cout << "Fill in color:--press enter to proceed>>" << endl;
       getchar();
      Joint_move_client.call(mv_srv2);
      
      //sleep for 2 seconds.
      wam_srvs::JointMove mv_srv3;
      mv_srv3.request.joints = convert2Float( getColorPose);
       cout << "Dig into:-->>" << endl;
       getchar();
      Joint_move_client.call(mv_srv3);
      
          wam_srvs::JointMove mv_srv4;
      mv_srv4.request.joints = convert2Float( leaveColorPoolPose);
       cout << "come to leave state:-->>" << endl;
       getchar();
      Joint_move_client.call(mv_srv4);
      
         wam_srvs::JointMove mv_srv5;
      mv_srv5.request.joints = convert2Float( previousJointState);
       cout << "come to stop state:-->>" << endl;
       getchar();
      Joint_move_client.call(mv_srv5);
      
	taskCount=0;
	 cout << "back to the task:-->>" << endl;
	 getchar();
      }
      
      cout<<"-----------task killed!"<<"---------move to next task-------------"<<endl;
      taskinProgress=false;
      	std_msgs::String msg;
	std::stringstream ss;
	ss << "t" ;
	msg.data = ss.str();
	pubTask.publish(msg);
	ros::spinOnce();
    }
    }
  
    loop_rate.sleep();
    
 }
  catch(vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
    }
    }
    
 task.kill();
}

