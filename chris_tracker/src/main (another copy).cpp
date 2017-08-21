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
#include <termios.h>

#define PI 3.14159265

using namespace std;

ros::Publisher pub_pd;
ros::Publisher pub_p;

//float cam_k[];
//float Cam_D[];
cv::Mat Hom;
double scaleFactor=0.3;
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
  std::vector< std::vector<cv::Point3d> > desiredPoints;
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

std::vector<cv::Point3d> drawingPoints;
bool firstPictureTaken=false;
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
	std::vector<cv::Point3d> lineSet;
         for(int j=0; j<poly_out[i].dim-1; j++ )
	 { 
	    double firstPointz=-1;
	   cv::Point2d p1st(poly_out[i].pts[j].x, poly_out[i].pts[j].y);
	   if(j==0) //the first point of line, set this point to be zero
	     firstPointz=0;
	   
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
	  bool firstrun=true;
	  while(xIndex<objectIndex) // scan using x
	  {
	    cv::Point2d firstP=getPointInLine(xIndex, p1st, p2nd);
	    xIndex+=incrementalX;
	    if(firstrun)
	       lineSet.push_back(cv::Point3d(firstP.x, firstP.y, firstPointz));
	    else
	      lineSet.push_back(cv::Point3d(firstP.x, firstP.y, -1));
	    firstrun=false;
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
	   bool firstrun=true;
	  while(yIndex<objectYIndex)
	  {
	      cv::Point2d firstP(p1st.x, yIndex);
	    yIndex+=incrementalX;
	     if(firstrun)
	       lineSet.push_back(cv::Point3d(firstP.x, firstP.y, firstPointz));
	    else
	      lineSet.push_back(cv::Point3d(firstP.x, firstP.y, -1));
	    firstrun=false;
	  }
	  }
	 }
       desiredPoints.push_back(lineSet);
      }
//manually define the points
// std::vector< std::vector<cv::Point3d> > originPlys;
// 
// //---------------------original-line------------------------0
// std::vector<cv::Point3d> lineSet0;
// lineSet0.push_back(cv::Point3d(77,51,0));
// lineSet0.push_back(cv::Point3d(251,51,0));
// lineSet0.push_back(cv::Point3d(251,306,0));
// //lineSet0.push_back(cv::Point3d(77,306,0));
// 
// originPlys.push_back(lineSet0);
// 
// //---------------------original-line------------------------1
// std::vector<cv::Point3d> lineSet1;
// lineSet1.push_back(cv::Point3d(78,51,0));
// lineSet1.push_back(cv::Point3d(77,306,0));
// lineSet1.push_back(cv::Point3d(251,306,0));
// 
// 
// originPlys.push_back(lineSet1);

// //---------------------original-line------------------------0
// std::vector<cv::Point3d> lineSet0;
// lineSet0.push_back(cv::Point3d(152.877,501.525,0));
// lineSet0.push_back(cv::Point3d(97.232,490.904,0));
// lineSet0.push_back(cv::Point3d(64.6079,483.902,0));
// lineSet0.push_back(cv::Point3d(16.2721,490.928,0));
// 
// originPlys.push_back(lineSet0);
// 
// //---------------------original-line------------------------1
// std::vector<cv::Point3d> lineSet1;
// lineSet1.push_back(cv::Point3d(292.782,424.625,0));
// lineSet1.push_back(cv::Point3d(266.768,420.717,0));
// lineSet1.push_back(cv::Point3d(230.523,424.737,0));
// originPlys.push_back(lineSet1);
//  
// //---------------------original-line------------------------2
// std::vector<cv::Point3d> lineSet2;
// lineSet2.push_back(cv::Point3d(351.673,491.647,0));
// lineSet2.push_back(cv::Point3d(307.468,482.658,0));
// lineSet2.push_back(cv::Point3d(220.531,497.672,0));
// lineSet2.push_back(cv::Point3d(192.284,505.767,0));
// originPlys.push_back(lineSet2);
//  
// //---------------------original-line------------------------3
// std::vector<cv::Point3d> lineSet3;
// lineSet3.push_back(cv::Point3d(351.664,491.447,0));
// lineSet3.push_back(cv::Point3d(382.69,496.278,0));
// lineSet3.push_back(cv::Point3d(424.835,508.316,0));
// originPlys.push_back(lineSet3);
//  
// //---------------------original-line------------------------4
// std::vector<cv::Point3d> lineSet4;
// lineSet4.push_back(cv::Point3d(77.4205,482.864,0));
// lineSet4.push_back(cv::Point3d(125.529,493.369,0));
// lineSet4.push_back(cv::Point3d(164.455,500.72,0));
// lineSet4.push_back(cv::Point3d(194.502,502.479,0));
// lineSet4.push_back(cv::Point3d(251.681,488.316,0));
// originPlys.push_back(lineSet4);
//  
// //---------------------original-line------------------------5
// std::vector<cv::Point3d> lineSet5;
//  
// lineSet5.push_back(cv::Point3d(265.495,417.538,0));
// lineSet5.push_back(cv::Point3d(294.759,421.659,0));
// lineSet5.push_back(cv::Point3d(225.591,421.636,0));
// originPlys.push_back(lineSet5);
//  
// //---------------------original-line------------------------7
// std::vector<cv::Point3d> lineSet7;
//  lineSet7.push_back(cv::Point3d(392.183,204.463,0));
// lineSet7.push_back(cv::Point3d(375.433,207.069,0));
// originPlys.push_back(lineSet7);
//  
// //---------------------original-line------------------------8
// std::vector<cv::Point3d> lineSet8;
//  lineSet8.push_back(cv::Point3d(173.531,232.328,0));
// lineSet8.push_back(cv::Point3d(139.195,226.215,0));
// lineSet8.push_back(cv::Point3d(111.647,220.421,0));
// lineSet8.push_back(cv::Point3d(74.4818,225.34,0));
// originPlys.push_back(lineSet8);
//  
// //---------------------original-line------------------------9
// std::vector<cv::Point3d> lineSet9;
//  lineSet9.push_back(cv::Point3d(172.491,416.508,0));
// lineSet9.push_back(cv::Point3d(171.63,313.499,0));
// lineSet9.push_back(cv::Point3d(173.442,233.478,0));
// originPlys.push_back(lineSet9);
//  
// //---------------------original-line------------------------11
// std::vector<cv::Point3d> lineSet11;
//  lineSet11.push_back(cv::Point3d(177.427,415.089,0));
// lineSet11.push_back(cv::Point3d(225.554,421.067,0));
// originPlys.push_back(lineSet11);
//  
// //---------------------original-line------------------------13
// std::vector<cv::Point3d> lineSet13;
//  lineSet13.push_back(cv::Point3d(388.516,199.492,0));
// lineSet13.push_back(cv::Point3d(348.814,120.342,0));
// lineSet13.push_back(cv::Point3d(324.776,87.303,0));
// lineSet13.push_back(cv::Point3d(289.234,56.8119,0));
// lineSet13.push_back(cv::Point3d(248.147,26.6587,0));
// lineSet13.push_back(cv::Point3d(209.379,6.7315,0));
// originPlys.push_back(lineSet13);
//  
// //---------------------original-line------------------------14
// std::vector<cv::Point3d> lineSet14;
//  lineSet14.push_back(cv::Point3d(180.957,211.423,0));
// lineSet14.push_back(cv::Point3d(175.304,251.472,0));
// lineSet14.push_back(cv::Point3d(175.83,268.5,0));
// originPlys.push_back(lineSet14);
//  
// //---------------------original-line------------------------15
// std::vector<cv::Point3d> lineSet15;
//  lineSet15.push_back(cv::Point3d(286.213,166.442,0));
// lineSet15.push_back(cv::Point3d(283.998,215.477,0));
// originPlys.push_back(lineSet15);
//  
// //---------------------original-line------------------------17
// std::vector<cv::Point3d> lineSet17;
//  lineSet17.push_back(cv::Point3d(259.271,277.484,0));
// lineSet17.push_back(cv::Point3d(262.488,231.429,0));
// lineSet17.push_back(cv::Point3d(263.106,370.517,0));
// originPlys.push_back(lineSet17);
//  
// //---------------------original-line------------------------20
// std::vector<cv::Point3d> lineSet20;
//  lineSet20.push_back(cv::Point3d(281.507,166.998,0));
// lineSet20.push_back(cv::Point3d(217.499,166.686,0));
// lineSet20.push_back(cv::Point3d(218.532,165.656,0));
// lineSet20.push_back(cv::Point3d(186.264,172.364,0));
// originPlys.push_back(lineSet20);
//  
// //---------------------original-line------------------------21
// std::vector<cv::Point3d> lineSet21;
//  lineSet21.push_back(cv::Point3d(66.2659,220.407,0));
// lineSet21.push_back(cv::Point3d(97.0005,143.301,0));
// lineSet21.push_back(cv::Point3d(96.6528,142.597,0));
// lineSet21.push_back(cv::Point3d(121.99,102.812,0));
// lineSet21.push_back(cv::Point3d(122.486,102.799,0));
// lineSet21.push_back(cv::Point3d(149.596,63.566,0));
// lineSet21.push_back(cv::Point3d(150.204,63.2281,0));
// lineSet21.push_back(cv::Point3d(189.453,20.4567,0));
// lineSet21.push_back(cv::Point3d(190.738,20.7842,0));
// lineSet21.push_back(cv::Point3d(205.949,8.03613,0));
// originPlys.push_back(lineSet21);
//  
// //---------------------original-line------------------------22
// std::vector<cv::Point3d> lineSet22;
//  lineSet22.push_back(cv::Point3d(73.696,224.763,0));
// lineSet22.push_back(cv::Point3d(67.5954,223.141,0));
// originPlys.push_back(lineSet22);
//  
// //---------------------original-line------------------------23
// std::vector<cv::Point3d> lineSet23;
//  lineSet23.push_back(cv::Point3d(208.085,359.498,0));
// lineSet23.push_back(cv::Point3d(208.669,249.501,0));
// originPlys.push_back(lineSet23);
//  
// //---------------------original-line------------------------24
// std::vector<cv::Point3d> lineSet24;
//  lineSet24.push_back(cv::Point3d(297.517,421.499,0));
// lineSet24.push_back(cv::Point3d(295.776,385.438,0));
// lineSet24.push_back(cv::Point3d(285.226,293.418,0));
// originPlys.push_back(lineSet24);
//  
// //---------------------original-line------------------------26
// std::vector<cv::Point3d> lineSet26;
//  lineSet26.push_back(cv::Point3d(281.543,215.503,0));
// lineSet26.push_back(cv::Point3d(281.39,171.497,0));
// originPlys.push_back(lineSet26);
//  
// //---------------------original-line------------------------28
// std::vector<cv::Point3d> lineSet28;
//  lineSet28.push_back(cv::Point3d(289.097,300.433,0));
// lineSet28.push_back(cv::Point3d(285.165,221.467,0));
// originPlys.push_back(lineSet28);
//  
// //---------------------original-line------------------------30
// std::vector<cv::Point3d> lineSet30;
//  lineSet30.push_back(cv::Point3d(286.33,216.314,0));
// lineSet30.push_back(cv::Point3d(387.543,201.802,0));
// originPlys.push_back(lineSet30);
//  
// //---------------------original-line------------------------31
// std::vector<cv::Point3d> lineSet31;
//  lineSet31.push_back(cv::Point3d(74.8341,208.262,0));
// lineSet31.push_back(cv::Point3d(70.7862,219.602,0));
// originPlys.push_back(lineSet31);
//  
// //---------------------original-line------------------------34
// std::vector<cv::Point3d> lineSet34;
//  lineSet34.push_back(cv::Point3d(74.367,207.446,0));
// lineSet34.push_back(cv::Point3d(89.7849,169.617,0));
// lineSet34.push_back(cv::Point3d(103.586,136.124,0));
// lineSet34.push_back(cv::Point3d(148.316,70.3747,0));
// lineSet34.push_back(cv::Point3d(191.663,22.6497,0));
// lineSet34.push_back(cv::Point3d(209.379,9.35201,0));
// originPlys.push_back(lineSet34);
 
//descretlize points 
//         double threshold=5;
// 	double incrementalX=5;
// 	for(int i=0; i<originPlys.size(); i++ )
//        {
// 	std::vector<cv::Point3d> lineSet;
// 	std::vector<cv::Point3d> originLineSet=originPlys[i];
//          for(int j=0; j<originLineSet.size()-1; j++ )
// 	 { 
// 	    double firstPointz=-1;
// 	   cv::Point2d p1st(originLineSet[j].x, originLineSet[j].y);
// 	   if(j==0) //the first point of line, set this point to be zero
// 	     firstPointz=0;
// 	   
// 	   cv::Point2d p2nd(originLineSet[j+1].x, originLineSet[j+1].y);
// 	   double distance=sqrt((p1st.x-p2nd.x)*(p1st.x-p2nd.x)+(p1st.y-p2nd.y)*(p1st.y-p2nd.y));
// 	  bool firstrun=true;
// 	   if(distance<threshold)
// 	   {
// 	      if(firstrun)
// 	       lineSet.push_back(cv::Point3d(p1st.x, p1st.y, firstPointz));
// 	    else
// 	      lineSet.push_back(cv::Point3d(p1st.x, p1st.y, -1));
// 	    firstrun=false;
// 	    lineSet.push_back(cv::Point3d(p2nd.x, p2nd.y, -1));
// 	  }
// 	  else
// 	  {
// 	     
// 	   
// 	     if(abs(p1st.x-p2nd.x)>5)
// 	     {
// 	        double objectIndex=p2nd.x;
// 	        double xIndex=p1st.x;
// 	        if(objectIndex<xIndex)
// 	          incrementalX=0-incrementalX;
// 	  
// 	         while(abs(xIndex-objectIndex)>threshold) // scan using x
// 	        {
// 	            cv::Point2d firstP=getPointInLine(xIndex, p1st, p2nd);
// 	            xIndex+=incrementalX;
// 	            if(firstrun)
// 	               lineSet.push_back(cv::Point3d(firstP.x, firstP.y, firstPointz));
// 	            else
// 	               lineSet.push_back(cv::Point3d(firstP.x, firstP.y, -1));
// 	           firstrun=false;
// 	         }
// 	      }
// 	      else //(p1st.x==p2nd.x)
// 	     {
// 	          //scan using y
// 	         double yIndex=p1st.y;
// 	         double objectYIndex=p2nd.y;
// 	         if(objectYIndex<yIndex)
// 	         incrementalX=0-incrementalX;
// 		 
// 	         bool firstrun=true;
// 	         while(abs(yIndex-objectYIndex)>threshold)
// 	        {
// 	            cv::Point2d firstP(p1st.x, yIndex);
// 	            yIndex+=incrementalX;
// 	            if(firstrun)
// 	                lineSet.push_back(cv::Point3d(firstP.x, firstP.y, firstPointz));
// 	            else
// 	                lineSet.push_back(cv::Point3d(firstP.x, firstP.y, -1));
// 	            firstrun=false;
// 	         }
// 	     }
// 	   }
// 	 }
//        desiredPoints.push_back(lineSet);
//       }


  
   cout<<"desiredPoint size:"<<desiredPoints.size()<<endl;
   
//    //write to files
//    ofstream file;////////
//     file.open("desiredPoints.txt",ios::app);
//     for(int i=0;i<desiredPoints.size();i++)
//     {
//       std::vector<cv::Point3d> lineSet=desiredPoints[i];
//       file<<"--------------------desired-line------------------------"<<i<<endl;
//       for(int j=0;j<lineSet.size();j++)
//       {
// 	std::ostringstream d2sx;
//         std::ostringstream d2sy;
// 	std::ostringstream d2sz;
// 	d2sx<<lineSet[j].x;
// 	d2sy<<lineSet[j].y;
// 	d2sz<<lineSet[j].z;
// 	 string xt=d2sx.str()+",";
//         string yt=d2sy.str()+",";
//         string zt=d2sz.str();
// 	 file<<xt;
//         file<<yt;
//         file<<zt<<endl;
//       }
//       file<<endl;
//     }
//     file.close();
//     
//     ofstream file2;////////
//     file2.open("originalPoints.txt",ios::app);
//    for(int i=0; i<originPlys.size(); i++ )
//     {
//       file2<<"---------------------original-line------------------------"<<i<<endl;
//       std::vector<cv::Point3d> originLineSet=originPlys[i];
//       for(int j=0; j<originLineSet.size(); j++ )
//       {
// 	std::ostringstream d2sx;
//         std::ostringstream d2sy;
// 	std::ostringstream d2sz;
// 	d2sx<<originLineSet[j].x;
// 	d2sy<<originLineSet[j].y;
// 	d2sz<<0;
// 	 string xt=d2sx.str()+",";
//         string yt=d2sy.str()+",";
//         string zt=d2sz.str();
// 	 file2<<xt;
//         file2<<yt;
//         file2<<zt<<endl;
//       }
//       file2<<endl;
//     }
//     file2.close();
}
// int getch()
// {
//   static struct termios oldt, newt;
//   tcgetattr( STDIN_FILENO, &oldt);           // save old settings
//   newt = oldt;
//   newt.c_lflag &= ~(ICANON);                 // disable buffering      
//   tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings
// 
//   int c = getchar();  // read character (non-blocking)
// 
//   tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
//   return c;
// }

 void imageCallback(const sensor_msgs::ImageConstPtr& msg)
   {
    try
     {
        cv::Mat myImage=cv_bridge::toCvShare(msg, "bgr8")->image;//using for simulation display
       if(firstPictureTaken)
       {
	 drawingInProcess=true;
       //cv::rectangle(ViewControl,centerStartP,centerEndP,cv::Scalar(0,0,255),2);
       if(validH)
       {
	 //show previous drawings, for simulation use only!!!!!!
	 for(int i=1;i<drawingPoints.size();i++)  //from line to line////////////////////only for simulation use, to display previous drawings.
	 {
	   cv::Point3d thisp=drawingPoints[i-1];
	    cv::Mat_<double> pt(3,1);
             pt(0,0)=thisp.x;
             pt(1,0)=thisp.y;
             pt(2,0)=1.0;
	     cv::Mat_<double> Pt1=Hom*pt; 
             cv::Point2d dstPt(Pt1(0,0)*scaleFactor/Pt1(2,0), Pt1(1,0)*scaleFactor/Pt1(2,0));//dstP1 can be used for IBVS, however, its frame is (x, y) frame, which is different from (row, column) frame
	     cv::Point2d imagePoint=conver2ImagePoint(dstPt,0);//conver2ImagePoint(dstPt,0);

	    //cv::circle( myImage, imagePoint, 1.0, cv::Scalar( 255, 0, 0 ), 1, 2 );//mark the desired point
	    //changed to draw line directly
	     cv::Point3d thisp2=drawingPoints[i];
	    cv::Mat_<double> pt2(3,1);
             pt2(0,0)=thisp2.x;
             pt2(1,0)=thisp2.y;
             pt2(2,0)=1.0;
	     cv::Mat_<double> Pt22=Hom*pt2; 
             cv::Point2d dstPt2(Pt22(0,0)*scaleFactor/Pt22(2,0), Pt22(1,0)*scaleFactor/Pt22(2,0));//dstP1 can be used for IBVS, however, its frame is (x, y) frame, which is different from (row, column) frame
	     cv::Point2d imagePoint2=conver2ImagePoint(dstPt2,0);//conver2ImagePoint(dstPt,0);
             if(thisp2.z!=0) //not the first point of another lineset
	     {
	       cv::line(myImage, imagePoint,imagePoint2, cv::Scalar( 255, 0, 0 ), 1);
// 	       cout<<"draw a line"<<endl;
// 	       cout<<"p1:"<<imagePoint<<endl;
// 	       cout<<"p2:"<<imagePoint2<<endl;
	     }
	 }
	 //draw polygons
	 //descritelize by points, along column direction, increase by 2 pixels.
	 std::vector<cv::Point3d> lineset=desiredPoints[LineIndex];
	 //desired point
	 cv::Point3d firstP=lineset[PointInLineIndex];
	 //cout<<"first point P"<<firstP<<endl;
	 drawingPoints.push_back(firstP);
	 //cout<<"drawing point size"<<drawingPoints.size()<<endl;
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
 	    if(firstP.z==0) // if(PointInLineIndex==0)
	    {  msg.z=0;
	      //cout<<"set p.z=0"<<endl;
	    }
            // Publish the message.
            pub_pd.publish(msg);
	    //draw the desired point
	     cv::Point2d imagePoint=conver2ImagePoint(dstP1,1);
	     //cout<<"desired Point:"<<imagePoint<<endl;
	     cv::circle( myImage, imagePoint, 6.0, cv::Scalar( 0, 0, 255 ), 2, 2 );//mark the desired point
	     //draw the current point
	     imagePoint=conver2ImagePoint(currentPoint,1);
	     cv::circle( myImage, imagePoint, 4.0, cv::Scalar( 0, 255, 0 ), 1, 2 );//mark the current point
	     
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
	 
	 drawingInProcess=false;
	 
       }//end of if homo valid
       }//end of if firstPictureTaken
       else
       {
	 cout << "Press any key to capture the image..." << endl;
         getchar();
	  cv::Mat frame=cv::imread("/home/vis/Documents/Chris/samples/tree.png");
	 parseELSD(frame);
	 firstPictureTaken=true;
      }
    
       cv::imshow("view", myImage);
       cv::waitKey(30);
     }//end of try
     catch (cv_bridge::Exception& e)
     {
       ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
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
	
	//parseELSD();
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



 
