#include <iostream>
#include <cstdlib>
#include <fstream>
#include <list>
#include <stack>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include "std_msgs/String.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "ar_track_alvar_msgs/AlvarMarker.h"
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
extern "C"{
#include "image.h"
#include "pgm.h"
#include "misc.h"
#include "svg.h"
#include "polygon.h"
#include "ring.h"
#include "elsdc.h"
}
#include <math.h>
#define PI 3.14159265

using namespace std;

ros::Publisher pub_pd;
ros::Publisher pub_p;

//float cam_k[];
//float Cam_D[];
cv::Mat Hom;
double scaleFactor=0.6;
bool validH=false;
std::vector<cv::Point2d> pts_dst1;
std::vector<cv::Point2d> pts_dst2;
std::vector<cv::Point2d> pts_dst3;
std::vector<cv::Point2d> pts_dst4;
double image_width=300;
double image_height=456;
bool taskFinished=false;

Polygon *poly_out=NULL;/* array containing the parameters of the detected
								  polygons; correlated with poly_labels, i.e. the i-th
								  element of ell_labels is the label of the pixels
								  supporting the polygon defined by the parameters
								  poly[i] */

Ring *ell_out = NULL;/* array containing the parameters of the detected
								  ellipses; correlated with ell_labels, i.e. the i-th
								  element of ell_labels is the label of the pixels
								  supporting the ellipse defined by the parameters
								  ell[i] */								  
 int poly_count = 0;  /* number of detected polygons */
 int ell_count = 0;   /* number of detected ellipses */
 
 
  cv::Point2d currentPoint(0,0);
  double currentZ;
  std::vector< std::vector<cv::Point2d> > desiredPoints;
  bool drawingInProcess=false;

std::string doubleToString(double db)
 {
	 std::ostringstream doubleStreams;
	 		     	doubleStreams<<db;
	 	return doubleStreams.str();
 }
 
 /**
  * Test subscribe topic callback.
  * 
  * */
 void chatterCallback(const std_msgs::String::ConstPtr& o_msg)
 {
   
    
    ROS_INFO("I heard: [%s]", o_msg->data.c_str());
 }
 
 void ar_pose_marker_callback(ar_track_alvar_msgs::AlvarMarkers req)
 {
   size_t ll=sizeof(req.markers[0]);
   //cout<<"size of marker"<<ll<<endl;
  // ll=_msize(req.markers);
  // cout<<"total size"<<ll<<endl;
   int sizemarker=req.markers.size();
   //cout<<"size of markers"<<sizemarker<<endl;
   if(!req.markers.empty())
   {
   for(int i=0;i<sizemarker;i++)
   {
  
     ar_track_alvar_msgs::AlvarMarker marker=req.markers[i];
      tf::Quaternion q(marker.pose.pose.orientation.x, marker.pose.pose.orientation.y, marker.pose.pose.orientation.z, marker.pose.pose.orientation.w);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      //ROS_INFO("roll, pitch, yaw=%1.2f  %1.2f  %1.2f", roll, pitch, yaw);
     
      int id=marker.id;
      int con=marker.confidence;
      geometry_msgs::Pose pose=marker.pose.pose;
      geometry_msgs::Point position=pose.position;
      //geometry_msgs::Quaternion quat=pose->orientation;//quaternions, four elements.
      double x=position.x;
      double y=position.y;
      double z=position.z;
//       //convert x, y, z to image plane
//       std::vector<cv::Point3d> objectPoints;
//       cv::Point3d pt(x,y,z);
//       objectPoints.push_back(pt);
//      // cout<<objectPoints[0].x<<endl;
//       //cout<<objectPoints[0].y<<endl;
//      // cout<<objectPoints[0].z<<endl;
//       
//       
//       cv::Mat intrisicMat(3,3,cv::DataType<double>::type);
//       //this is for kinect
//       intrisicMat.at<double>(0,0)=570.3422241210938;
//       intrisicMat.at<double>(0,2)=319.5;
//       intrisicMat.at<double>(1,1)=570.3422241210938;
//       intrisicMat.at<double>(1,2)=239.5;
//       intrisicMat.at<double>(2,2)=1;
//       //poor camera
// //       intrisicMat.at<double>(0,0)=929.996179;
// //       intrisicMat.at<double>(0,2)=300.952418;
// //       intrisicMat.at<double>(1,1)=931.956932;
// //       intrisicMat.at<double>(1,2)=251.008379;
// //       intrisicMat.at<double>(2,2)=1;
//       
//       cv::Mat rVec(3, 1, cv::DataType<double>::type); // Rotation vector, all set to 0
//       rVec.at<double>(0) = 0;
//       rVec.at<double>(1) =0;
//       rVec.at<double>(2) =0;
//       cv::Mat tVec(3, 1, cv::DataType<double>::type); // Translation vector, all set to 0
//       tVec.at<double>(0) =0;
//       tVec.at<double>(1) =0;
//       tVec.at<double>(2) =0;
//       cv::Mat distCoeffs(5, 1, cv::DataType<double>::type);   // Distortion vector, all set to 0
//       distCoeffs.at<double>(0) =0;// 0.244575;
// distCoeffs.at<double>(1) = 0;//-0.170874;
// distCoeffs.at<double>(2) = 0;//-0.016775;
// distCoeffs.at<double>(3) = 0;//0.031988;
// distCoeffs.at<double>(4) = 0;
// 
//       std::vector<cv::Point2d> imagePoints;
//       cv::projectPoints(objectPoints, rVec, tVec, intrisicMat, distCoeffs, imagePoints);
      double dx;
      double dy;
      double dz=z;
      //if(!imagePoints.empty())
      {
	dx=x;//dx=imagePoints[0].x;
	dy=y;//dy=imagePoints[0].y;
	switch(id)
	{
	  case 1:
	    pts_dst1.push_back(cv::Point2d(dx,dy));
	    break;
	  case 2:
	    pts_dst2.push_back(cv::Point2d(dx,dy));
	    break;
	  case 3:
	    pts_dst3.push_back(cv::Point2d(dx,dy));
	    break;
	  case 4:
 	    pts_dst4.push_back(cv::Point2d(dx,dy));
	    break;
	  case 5:
	    currentPoint.x=dx;
	    currentPoint.y=dy;
	    currentZ=dz;
	    geometry_msgs::Point msg;
            msg.x = dx;
            msg.y = dy;
            msg.z=0;
            //Publish the message.
           pub_p.publish(msg);
	    break;
	}
      }
    /*for (unsigned int i = 0; i < projectedPoints.size(); ++i)
    {
        std::cout << "Image point: " << imagePoints[i] << " Projected to " << projectedPoints[i] << std::endl;
    }*/
      //cout<<id<<endl;
      //cout<<dx<<endl;
      //cout<<dy<<endl;
      //cout<<dx<<endl;
   }
   }//end if
   //update homography
   //check vector size to be >=100
//    cout<<"p2"<<pts_dst2.size()<<endl;
//    cout<<"p3"<<pts_dst3.size()<<endl;
//    cout<<"p4"<<pts_dst4.size()<<endl;
   if(pts_dst1.size()>1&&pts_dst2.size()>1&&pts_dst3.size()>1&&pts_dst4.size()>1)
   {
     std::vector<cv::Point2d> pts_dst;
     //calculate averge.////////////////////////////////////////////////later
     //check std. ///////////////////////////////////////////////////// later
     pts_dst.push_back(pts_dst1.at(0));
     pts_dst.push_back(pts_dst2.at(0));
     pts_dst.push_back(pts_dst3.at(0));
     pts_dst.push_back(pts_dst4.at(0));
     
     std::vector<cv::Point2d> pts_src;
     pts_src.push_back(cv::Point2d(0,0));
     pts_src.push_back(cv::Point2d(image_width,0));
     pts_src.push_back(cv::Point2d(0,image_height));
     pts_src.push_back(cv::Point2d(image_width,image_height));
     
     Hom=cv::findHomography(pts_src,pts_dst,CV_RANSAC);
     
     validH=true;
     cout<<"homography set done!"<<endl;
     pts_dst1.clear();
     pts_dst2.clear();
     pts_dst3.clear();
     pts_dst4.clear();

  }
   //after homography, clear the vector
   //after homography, set validH to be true.
 }
 
 cv::Point2d estimatePointInEllipse(double angle, double a, double b, double theta, double cx, double cy)
 {
   
   //firstly, get ax, by.
   double tant=tan(angle);
   double down=sqrt(b*b+a*a*tant*tant);
   
   double ax=a*b/down;
   if(angle>=1.5*PI&&angle<=2*PI)
     ax=-ax;
   double by=ax*tant;
   //from ax, by to x, y
   double x=ax*cos(theta)-by*sin(theta)+cx;
   double y=ax*sin(theta)+by*cos(theta)+cy;
   
   cv::Point2d resultP(x,y);
   return resultP;
}

cv::Point2d getPointInLine(double x, cv::Point2d p1, cv::Point2d p2) //x1 !=x2;
{
  double deltaY=p2.y-p1.y;
  double deltaX=p2.x-p1.x;
  double deltaNX=x-p1.x;
  if(p2.x==p1.x)
    return cv::Point2d(x,p1.y);
  else
  {
    double y=deltaY*deltaNX/deltaX+p1.y;
    return cv::Point2d(x,y);
  } 
   
}

int LineIndex=0;
int PointInLineIndex=0;

cv::Point2d conver2ImagePoint(cv::Point2d camPoint, int _type)
{
    //convert x, y, z to image plane
      cv::Point3d ps(camPoint.x, camPoint.y, 0.77);
      if(_type==1)
	ps.z=currentZ;
      std::vector<cv::Point3d> objectPoints;
      objectPoints.push_back(ps);
     // cout<<objectPoints[0].x<<endl;
      //cout<<objectPoints[0].y<<endl;
     // cout<<objectPoints[0].z<<endl;
      
      
      cv::Mat intrisicMat(3,3,cv::DataType<double>::type);
      //this is for kinect
      intrisicMat.at<double>(0,0)=570.3422241210938;
      intrisicMat.at<double>(0,2)=319.5;
      intrisicMat.at<double>(1,1)=570.3422241210938;
      intrisicMat.at<double>(1,2)=239.5;
      intrisicMat.at<double>(2,2)=1;
      //poor camera
//       intrisicMat.at<double>(0,0)=929.996179;
//       intrisicMat.at<double>(0,2)=300.952418;
//       intrisicMat.at<double>(1,1)=931.956932;
//       intrisicMat.at<double>(1,2)=251.008379;
//       intrisicMat.at<double>(2,2)=1;
      
      cv::Mat rVec(3, 1, cv::DataType<double>::type); // Rotation vector, all set to 0
      rVec.at<double>(0) = 0;
      rVec.at<double>(1) =0;
      rVec.at<double>(2) =0;
      cv::Mat tVec(3, 1, cv::DataType<double>::type); // Translation vector, all set to 0
      tVec.at<double>(0) =0;
      tVec.at<double>(1) =0;
      tVec.at<double>(2) =0;
      cv::Mat distCoeffs(5, 1, cv::DataType<double>::type);   // Distortion vector, all set to 0
      distCoeffs.at<double>(0) =0;// 0.244575;
distCoeffs.at<double>(1) = 0;//-0.170874;
distCoeffs.at<double>(2) = 0;//-0.016775;
distCoeffs.at<double>(3) = 0;//0.031988;
distCoeffs.at<double>(4) = 0;

      std::vector<cv::Point2d> imagePoints;
      cv::projectPoints(objectPoints, rVec, tVec, intrisicMat, distCoeffs, imagePoints);
      return imagePoints[0];
}

cv::Point2d homographyConver2ImagePoint(cv::Point2d firstPoint, int marki)
{
   cv::Mat_<double> pt(3,1);
             pt(0,0)=firstPoint.x;
             pt(1,0)=firstPoint.y;
             pt(2,0)=1.0;
	     cv::Mat_<double> Pt1=Hom*pt; 
             cv::Point2d dstPt(Pt1(0,0)*scaleFactor/Pt1(2,0), Pt1(1,0)*scaleFactor/Pt1(2,0));//dstP1 can be used for IBVS, however, its frame is (x, y) frame, which is different from (row, column) frame
	     cv::Point2d imagePoint=conver2ImagePoint(dstPt,marki);
	     return imagePoint;
}

std::vector<cv::Point2d> drawingPoints;
 void imageCallback(const sensor_msgs::ImageConstPtr& msg)
   {
    try
     {
       //if(!drawingInProcess)
       {
	 drawingInProcess=true;
       cv::Mat myImage=cv_bridge::toCvShare(msg, "bgr8")->image;//using for simulation display
       //cv::rectangle(ViewControl,centerStartP,centerEndP,cv::Scalar(0,0,255),2);
       if(validH)
       {
	 //show previous drawings, for simulation use only!!!!!!
	 for(int i=0;i<drawingPoints.size();i++)  //from line to line////////////////////only for simulation use, to display previous drawings.
	 {
	   cv::Point2d thisp=drawingPoints[i];
	    cv::Mat_<double> pt(3,1);
             pt(0,0)=thisp.x;
             pt(1,0)=thisp.y;
             pt(2,0)=1.0;
	     cv::Mat_<double> Pt1=Hom*pt; 
             cv::Point2d dstPt(Pt1(0,0)*scaleFactor/Pt1(2,0), Pt1(1,0)*scaleFactor/Pt1(2,0));//dstP1 can be used for IBVS, however, its frame is (x, y) frame, which is different from (row, column) frame
	     cv::Point2d imagePoint=conver2ImagePoint(dstPt,0);

	    cv::circle( myImage, imagePoint, 1.0, cv::Scalar( 255, 0, 0 ), 1, 2 );//mark the desired point
	    //changed to draw line directly
	 }
	 
// 	 //show previous drawing, using lines
// 	 for(int i=0;i<LineIndex;i++)
// 	 {
// 	   std::vector<cv::Point2d> thislineset=desiredPoints[i];
// 	   cv::Point2d firstPoint=thislineset[0];
// 	   cv::Point2d secondPoint=thislineset[1];
// 	   cv::Point2d imp1= homographyConver2ImagePoint(firstPoint,0);
// 	   cv::Point2d imp2=homographyConver2ImagePoint(secondPoint,0);
// 	   cv::line(myImage, imp1,imp2, cv::Scalar( 255, 0, 0 ), 2);
// 	}
	 
	 //draw polygons
	 //descritelize by points, along column direction, increase by 2 pixels.
	 std::vector<cv::Point2d> lineset=desiredPoints[LineIndex];
	 
	 cv::Point2d firstP=lineset[PointInLineIndex];
	 drawingPoints.push_back(firstP);
	     cv::Mat_<double> p1(3,1);
             p1(0,0)=firstP.x;
             p1(1,0)=firstP.y;
             p1(2,0)=1.0;
	     cv::Mat_<double> P21=Hom*p1;
             cv::Point2d dstP1(P21(0,0)*scaleFactor/P21(2,0), P21(1,0)*scaleFactor/P21(2,0));//dstP1 can be used for IBVS, however, its frame is (x, y) frame, which is different from (row, column) frame
	    geometry_msgs::Point msg;
            msg.x = dstP1.x;
            msg.y = dstP1.y;
            msg.z=-1;
	    if(PointInLineIndex==0)
	      msg.z=0;
            // Publish the message.
            pub_pd.publish(msg);
	     cv::Point2d imagePoint=conver2ImagePoint(dstP1,1);
	     //cout<<"desired Point:"<<imagePoint<<endl;
	     cv::circle( myImage, imagePoint, 6.0, cv::Scalar( 0, 0, 255 ), 2, 2 );//mark the desired point
// 	     //estimate the 3D point
// 	          cv::Mat intrisicMat(3,3,cv::DataType<double>::type);
//       intrisicMat.at<double>(0,0)=570.3422241210938;
//       intrisicMat.at<double>(0,2)=319.5;
//       intrisicMat.at<double>(1,1)=570.3422241210938;
//       intrisicMat.at<double>(1,2)=239.5;
//       intrisicMat.at<double>(2,2)=1;
//       cv::Mat projectionMatrix(3,4, cv::DataType<double>::type);
//       projectionMatrix.at<double>(0,0)=1;
//       projectionMatrix.at<double>(1,1)=1;
//       projectionMatrix.at<double>(2,2)=1;
//       
//       cv::Mat externalMatrix(4, 4, cv::DataType<double>::type); // Rotation vector, all set to 0
//       externalMatrix.at<double>(0,0) = -0.07149830271;
//       externalMatrix.at<double>(0,1) =-0.409727361;
//      externalMatrix.at<double>(0,2) =0.9094017167;
//       externalMatrix.at<double>(0,3) =-0.3320618852;
//       
//        externalMatrix.at<double>(1,0) =-0.9931621713;
//         externalMatrix.at<double>(1,1) =0.07203075161;
// 	 externalMatrix.at<double>(1,2) =-0.09187204276;
// 	  externalMatrix.at<double>(1,3) =0.4548481991;
// 	  
// 	   externalMatrix.at<double>(2,0) =-0.05703951667;
// 	    externalMatrix.at<double>(2,1) =-0.8979247162;
// 	     externalMatrix.at<double>(2,2) =-0.4364375071;
// 	      externalMatrix.at<double>(2,3) =0.3257839661;
//       
//       externalMatrix.at<double>(3,3) =1;
//       
//       cv::Mat cameraP=intrisicMat*projectionMatrix*externalMatrix;
//       cv::Mat p2d(3,1,cv::DataType<double>::type);
//       p2d.at<double>(0,0)=dstP1.x;
//       p2d.at<double>(1,0)=dstP1.y;
//       p2d.at<double>(2,0)=1;
//       
      //cv::Mat p3d=cameraP.inv()*p2d;
      
      //cout<<"p3d"<<p3d<<endl;
	     imagePoint=conver2ImagePoint(currentPoint,1);
	     cv::circle( myImage, imagePoint, 4.0, cv::Scalar( 0, 255, 0 ), 1, 2 );//mark the current point
	     // do visp, ibvs
	     //cv::waitKey(0);
	     
       
	     //cout<<"drawing accomplished!"<<endl;
	     if(taskFinished) //
	     {
	     PointInLineIndex++;
	     if(PointInLineIndex>=lineset.size())//the end of the current line
	     {  
	       PointInLineIndex=0;
	        LineIndex++;
	        if(LineIndex>=desiredPoints.size())//the end of the current task
		{    
		  LineIndex=0;
		  drawingPoints.clear();
		}
	     }
	     taskFinished=false;
	     }
// 	 for(int i=0;i<desiredPoints.size();i++)  //from line to line
// 	 {
// 	   std::vector<cv::Point2d> lineset=desiredPoints[i];
// // 	   for(int j=0;j<lineset.size()-1;j++)  //from point to point  /////////////drawing simulation
// // 	   {
// // 	     cv::Point2d firstP=lineset[j];
// // 	     cv::Point2d secondP=lineset[j+1];
// // 	     //drawing /////////////////////////simulations
// // 	       cv::Mat_<double> p1(3,1);
// //      p1(0,0)=firstP.x;
// //      p1(1,0)=firstP.y;
// //      p1(2,0)=1.0;
// //     cv::Mat_<double> p2(3,1);
// //      p2(0,0)=secondP.x;
// //      p2(1,0)=secondP.y;
// //      p2(2,0)=1.0;
// // 	       cv::Mat_<double> P21=Hom*p1;
// //      cv::Mat_<double> P22=Hom*p2;
// //       cv::Point2d dstP1(P21(0,0)/P21(2,0), P21(1,0)/P21(2,0));
// //      cv::Point2d dstP2(P22(0,0)/P22(2,0), P22(1,0)/P22(2,0));
// //       cv::line(myImage,dstP1, dstP2,cv::Scalar(255,0,0),1);  
// // 	     
// // 	  }
// 	   for(int j=0;j<lineset.size()-1;j++)  //from point to point //drawing using visp
// 	   {
// 	     cv::Point2d firstP=lineset[j];
// 	     cv::Mat_<double> p1(3,1);
//              p1(0,0)=firstP.x;
//              p1(1,0)=firstP.y;
//              p1(2,0)=1.0;
// 	     cv::Mat_<double> P21=Hom*p1;
//              cv::Point2d dstP1(P21(0,0)/P21(2,0), P21(1,0)/P21(2,0));//dstP1 can be used for IBVS, however, its frame is (x, y) frame, which is different from (row, column) frame
// 	     
// 	     cv::circle( myImage, dstP1, 2.0, cv::Scalar( 0, 0, 255 ), 1, 2 );//mark the desired point
// 	     // do visp, ibvs
// 	     //cv::waitKey(0);
// 	      cv::imshow("view", myImage);
//        
// 	     cout<<"drawing accomplished!"<<endl;
// 
// 	  }
// 	   
// 	}
	 
	 drawingInProcess=false;
	 
// 	 if( poly_out != NULL )
//    {
//      for(int i=0; i<poly_count; i++ )
//        {
//          for(int j=0; j<poly_out[i].dim-1; j++ )
// 	 { 
// 	   cv::Point2d p1st(poly_out[i].pts[j].x, poly_out[i].pts[j].y);
// 	   cv::Point2d p2nd(poly_out[i].pts[j+1].x, poly_out[i].pts[j+1].y);
// 	  
// 	   double incrementalX=1;
// 	   if(abs(p1st.x-p2nd.x)>5)
// 	   {
// 	   double objectIndex=p2nd.x;
// 	    double xIndex=p1st.x;
// 	   if(objectIndex<xIndex)
// 	   {
// 	     xIndex=p2nd.x;
// 	     objectIndex=p1st.x;
// 	  }
// 	  while(xIndex<objectIndex) // scan using x
// 	  {
// 	    cv::Point2d firstP=getPointInLine(xIndex, p1st, p2nd);
// 	    xIndex+=incrementalX;
// 	    cv::Point2d secondP=getPointInLine(xIndex, p1st, p2nd);
// 	       cv::Mat_<double> p1(3,1);
//      p1(0,0)=firstP.x;
//      p1(1,0)=firstP.y;
//      p1(2,0)=1.0;
//     cv::Mat_<double> p2(3,1);
//      p2(0,0)=secondP.x;
//      p2(1,0)=secondP.y;
//      p2(2,0)=1.0;
// 	       cv::Mat_<double> P21=Hom*p1;
//      cv::Mat_<double> P22=Hom*p2;
//       cv::Point2d dstP1(P21(0,0)/P21(2,0), P21(1,0)/P21(2,0));
//      cv::Point2d dstP2(P22(0,0)/P22(2,0), P22(1,0)/P22(2,0));
//       cv::line(myImage,dstP1, dstP2,cv::Scalar(255,0,0),1);  
// 	  }
// 	   }
// 	  else //(p1st.x==p2nd.x)
// 	  {
// 	   //scan using y
// 	   double yIndex=p1st.y;
// 	   double objectYIndex=p2nd.y;
// 	   if(objectYIndex<yIndex)
// 	   {
// 	     yIndex=p2nd.y;
// 	     objectYIndex=p1st.y;
// 	  }
// 	  while(yIndex<objectYIndex)
// 	  {
// 	      cv::Point2d firstP(p1st.x, yIndex);
// 	    yIndex+=incrementalX;
// 	    cv::Point2d secondP(p1st.x, yIndex);
// 	       cv::Mat_<double> p1(3,1);
//      p1(0,0)=firstP.x;
//      p1(1,0)=firstP.y;
//      p1(2,0)=1.0;
//     cv::Mat_<double> p2(3,1);
//      p2(0,0)=secondP.x;
//      p2(1,0)=secondP.y;
//      p2(2,0)=1.0;
// 	       cv::Mat_<double> P21=Hom*p1;
//      cv::Mat_<double> P22=Hom*p2;
//       cv::Point2d dstP1(P21(0,0)/P21(2,0), P21(1,0)/P21(2,0));
//      cv::Point2d dstP2(P22(0,0)/P22(2,0), P22(1,0)/P22(2,0));
//       cv::line(myImage,dstP1, dstP2,cv::Scalar(255,0,0),1);  
// 	  }
// 	  }
// 	 }
//        }
//     
//    }
	 
//  if( ell_out != NULL )
//    {
//      //ell_count
//      for(int i=0; i<poly_count; i++ )
//        {
// 	// if(ell_out[i].theta!=0&&(ell_out[i].y1-ell_out[i].y2)>40)
// 	 {
// 
//       cout<<"ellipse Parameters:"<<poly_out[i].x1<<","<<poly_out[i].y1<<","<<ell_out[i].x2<<","<<ell_out[i].y2<<","<<ell_out[i].cx<<","<<ell_out[i].cy<<","<<
//       ell_out[i].ax<<","<<ell_out[i].bx<<","<<ell_out[i].theta<<","<<ell_out[i].ang_start<<","<<ell_out[i].ang_end<<endl;
//        double startAngle=ell_out[i].ang_start-ell_out[i].theta;
//        double endAngle=ell_out[i].ang_end-ell_out[i].theta;
//        if(startAngle<0)
// 	 startAngle+=2*PI;
//        if(endAngle<0)
// 	 endAngle+=2*PI;
//        cout<<"startAngle"<<startAngle*180/PI<<endl;
//         cout<<"endAngle"<<endAngle*180/PI<<endl;
//        //start Drawing using angle intervals.
//        double AngleIndex=startAngle;
//        double incrementalAngle=0.01;
//        double ObjectAngle=endAngle;
//        if(startAngle>endAngle)
//        {
// 	 AngleIndex=endAngle;
// 	 ObjectAngle=startAngle;
//       }
//       incrementalAngle=ObjectAngle-AngleIndex;
//       while(AngleIndex<ObjectAngle)
//       {
//        cv::Point2d firstPoint=estimatePointInEllipse(AngleIndex,ell_out[i].ax,ell_out[i].bx,ell_out[i].theta,ell_out[i].cx,ell_out[i].cy);
//        cout<<"NowAngle"<<AngleIndex*180/PI<<"   first Point:"<<firstPoint.x<<","<<firstPoint.y<<endl;
// 	AngleIndex+=incrementalAngle;
// 	cv::Point2d secondPoint=estimatePointInEllipse(AngleIndex,ell_out[i].ax,ell_out[i].bx,ell_out[i].theta,ell_out[i].cx,ell_out[i].cy);
// 	 cout<<"NowAngle"<<AngleIndex*180/PI<<"   second Point:"<<secondPoint.x<<","<<secondPoint.y<<endl;
// 	 
// 		 	   cv::Mat_<double> p1(3,1);	   
//      p1(0,0)=firstPoint.x;
//      p1(1,0)=firstPoint.y;
//      p1(2,0)=1.0;
//     cv::Mat_<double> p2(3,1);
//      p2(0,0)=secondPoint.x;
//      p2(1,0)=secondPoint.y;
//      p2(2,0)=1.0;
// 	       cv::Mat_<double> P21=Hom*p1;
//      cv::Mat_<double> P22=Hom*p2;
//       cv::Point2d dstP1(P21(0,0)/P21(2,0), P21(1,0)/P21(2,0));
//      cv::Point2d dstP2(P22(0,0)/P22(2,0), P22(1,0)/P22(2,0));
//       cv::line(myImage,dstP1, dstP2,cv::Scalar(255,0,0),1);
//       }
//        
//        }
//        }
//    }
   
       }//end of if homo valid
   
       cv::imshow("view", myImage);
       cv::waitKey(30);
       }
     }//end of try
     catch (cv_bridge::Exception& e)
     {
       ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
     }
     
  }
  
 
 
 
 PImageDouble read_pgm_image_double1(cv::Mat cv_image )
{
	PImageDouble image;
	/* get memory */
	int xsize=cv_image.size().width;
	int ysize=cv_image.size().height;
	  image = new_PImageDouble( xsize, ysize );

	  /* read data */

	  for(int y=0; y<ysize; y++)
	    for(int x=0; x<xsize; x++)
	    {
	    	cv::Scalar intensity=cv_image.at<uchar>(y, x);
	      image->data[ x + y * xsize ] = intensity.val[0];
	    }


	  return image;

}

 
void parseELSD(cv::Mat frame)
{
    //cv::Mat frame=cv::imread("/home/vis/Documents/Chris/samples/sample2.png");
    cv::Mat Src_L;
  cv::cvtColor( frame, Src_L, cv::COLOR_BGR2GRAY );
 
  cv::blur(Src_L, Src_L, cv::Size(3,3));//blur the image firstly, can be removed for test purpose.

		PImageDouble in;     /* input image */
		  PImageInt    out;    /* output image having the same size as 'in'; the pixels
								  supporting a certain geometric primitive are marked
								  with the same label */

		  
		  int *ell_labels=NULL;/* the pixels supporting a certain ellipse are marked
								  with the same unique label */
		  

		 
		  int *poly_labels=NULL;/* the pixels supporting a certain polygon are marked
								  with the same unique label */
		  

		  FILE *ell_ascii;     /* output file with the parameters of the detected
								  ellipses -- ASCII format */
		  FILE *poly_ascii;    /* output file with the parameters of the detected
								  polygons -- ASCII format */
		  FILE *fsvg;          /* output file with the detected ellipses and polygons
								  in vectorial form */
		  int i,j;

		  /* read input image; must be PGM form */
		  in = read_pgm_image_double1(Src_L);
		  int xsize = in->xsize, ysize = in->ysize;

		  /* create and initialize with 0 output label image */
		  out = new_PImageInt_ini( in->xsize, in->ysize, 0 );

		  /* call detection procedure */
		  ELSDc( in, &ell_count, &ell_out, &ell_labels, &poly_count, &poly_out,
				 &poly_labels, out );
		  
  //discreetlize points
	for(int i=0; i<poly_count; i++ )
       {
	std::vector<cv::Point2d> lineSet;
         for(int j=0; j<poly_out[i].dim-1; j++ )
	 { 
	    
	   cv::Point2d p1st(poly_out[i].pts[j].x, poly_out[i].pts[j].y);
	   cv::Point2d p2nd(poly_out[i].pts[j+1].x, poly_out[i].pts[j+1].y);
	   double incrementalX=10;
	   
	   if(abs(p1st.x-p2nd.x)>5)
	   {
	   double objectIndex=p2nd.x;
	    double xIndex=p1st.x;
	   if(objectIndex<xIndex)
	   {
	     xIndex=p2nd.x;
	     objectIndex=p1st.x;
	  }
	  while(xIndex<objectIndex) // scan using x
	  {
	    cv::Point2d firstP=getPointInLine(xIndex, p1st, p2nd);
	    xIndex+=incrementalX;
	     lineSet.push_back(firstP);
	  }
	   }
	  else //(p1st.x==p2nd.x)
	  {
	   //scan using y
	   double yIndex=p1st.y;
	   double objectYIndex=p2nd.y;
	   if(objectYIndex<yIndex)
	   {
	     yIndex=p2nd.y;
	     objectYIndex=p1st.y;
	  }
	  while(yIndex<objectYIndex)
	  {
	      cv::Point2d firstP(p1st.x, yIndex);
	    yIndex+=incrementalX;
	    lineSet.push_back(firstP);
	  }
	  }
	 }
       desiredPoints.push_back(lineSet);
	 
      }
       
}
 
 void nextTask_callback(const std_msgs::String::ConstPtr& msg)
 {
   taskFinished=true;
   if(msg->data=="reset")
   { 
     LineIndex=0;
     PointInLineIndex=0;
   }
}
 
int main( int argc, char **argv )
{
	//initialize the ROS system and become a node.
	ros::init(argc, argv, "chris_detection");
	ros::NodeHandle nh;
        
	//create a subscriber
	
	parseELSD();
	cout<<"Shape detector accomplished"<<endl;
	
	ros::Subscriber sub = nh.subscribe("/ar_pose_marker", 1000, ar_pose_marker_callback);
	ros::Subscriber sub2 = nh.subscribe("/chris_tracker/taskFinished", 1000, nextTask_callback);
        pub_pd = nh.advertise<geometry_msgs::Point>("/chris_tracker/desiredPoint", 1);
	pub_p = nh.advertise<geometry_msgs::Point>("/chris_tracker/currentPoint", 1);
	
//  while(ros::ok()) 
// {
// 
//     // Create and fill in the message.  The other four
//     // fields, which are ignored by turtlesim, default to 0.
//     geometry_msgs::Twist msg;
//     msg.linear.x = lx;
//     msg.angular.z = lz;
// 
//     // Publish the message.
//     pub.publish(msg);
//    
//     // Send a message to rosout with the details.
//     ROS_INFO_STREAM("Sending random velocity command:"
//       << " linear=" << msg.linear.x
//       << " angular=" << msg.angular.z);
// 
//     // Wait until it's time for another iteration.
//     rate.sleep();
//   }

     cv::namedWindow("view");
     cv::startWindowThread();
     image_transport::ImageTransport it(nh);
     image_transport::Subscriber subImage = it.subscribe("/camera/rgb/image_raw", 1, imageCallback);
    
  
	//loop at 100Hz unit the node is shutdown. actual rate is determined by computation speed.
	ros::spin();
    //declaration of function
	cv::destroyWindow("view");
	return 0;
}



 
