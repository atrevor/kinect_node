/*  kinect_node - a ROS node for publishing data from the kinect, using Hector Martin's awesome libfreenect

Copyright (C) 2010  Alex J. Trevor (atrevor@gmail.com) John G. Rogers III (jgrogers@gmail.com)

This code is licensed to you under the terms of the GNU GPL, version 2 or version 3;
see:
 http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt
 http://www.gnu.org/licenses/gpl-3.0.txt
*/

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <libusb.h>

extern "C" {
#include "libfreenect.h"
}

//#include <pthread.h>

//#include <GL/glut.h>
//#include <GL/gl.h>
//#include <GL/glu.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

#include <pcl/io/io.h>
#include <pcl/point_types.h>

#include "sensor_msgs/Image.h"

boost::mutex backbuf_mutex;
boost::condition cond;

struct point2 {
  double u, v;
point2(double u_, double v_): u(u_),v(v_){}
  bool operator == (const point2& rhs) const {
    double tol=1e-3;
    return (fabs(u-rhs.u)<tol && fabs(v - rhs.v)<tol);
  }
  point2 operator-(const point2& rhs) const {
    return point2(u-rhs.u, v-rhs.v);
  }
  double length2() const{ 
    return u*u + v*v;
  }
  double length() const {
    return sqrt(length());
  }
  
  void Print(const std::string& str) {
    std::cout << str <<" : ("<<u<<", "<<v<<")\n";
  }
};


uint8_t gl_depth_front[640*480*4];
uint8_t gl_depth_back[640*480*4];

uint8_t gl_rgb_front[640*480*4];
uint8_t gl_rgb_back[640*480*4];

uint16_t depths[640*480*sizeof(uint16_t)];

int got_frames;

int gl_depth_tex;
int gl_rgb_tex;

uint16_t t_gamma[2048];





  //From Hector
void depthimg(uint16_t *buf, int width, int height)
{
  boost::mutex::scoped_lock lock(backbuf_mutex);
  /*	FILE *f = fopen("depth.bin", "w");
	fwrite(depth_frame, 640*480, 2, f);
	fclose(f);*/
  
  //memcpy(depths,buf,640*480*sizeof(uint16_t));
  
  int i;
  
  //pthread_mutex_lock(&gl_backbuf_mutex);
  for (i=0; i<640*480; i++) {
    int pval = t_gamma[buf[i]];
    int lb = pval & 0xff;
    switch (pval>>8) {
    case 0:
      depths[i] = lb;
      gl_depth_back[3*i+0] = 255;
      gl_depth_back[3*i+1] = 255-lb;
      gl_depth_back[3*i+2] = 255-lb;
      break;
    case 1:
      depths[i] = 256 + lb;
      gl_depth_back[3*i+0] = 255;
      gl_depth_back[3*i+1] = lb;
      gl_depth_back[3*i+2] = 0;
      break;
    case 2:
      depths[i] = 512 + lb;
      gl_depth_back[3*i+0] = 255-lb;
      gl_depth_back[3*i+1] = 255;
      gl_depth_back[3*i+2] = 0;
      break;
    case 3:
      depths[i] = 768 + lb;
      gl_depth_back[3*i+0] = 0;
      gl_depth_back[3*i+1] = 255;
      gl_depth_back[3*i+2] = lb;
      break;
    case 4:
      depths[i] = 1024 + lb;
      gl_depth_back[3*i+0] = 0;
      gl_depth_back[3*i+1] = 255-lb;
      gl_depth_back[3*i+2] = 255;
      break;
    case 5:
      depths[i] = 1280 + lb;
      gl_depth_back[3*i+0] = 0;
      gl_depth_back[3*i+1] = 0;
      gl_depth_back[3*i+2] = 255-lb;
      break;
    default:
      depths[i] = 0;
      gl_depth_back[3*i+0] = 0;
      gl_depth_back[3*i+1] = 0;
      gl_depth_back[3*i+2] = 0;
      break;
    }
  }
  got_frames++;
  cond.notify_one();
  //pthread_cond_signal(&gl_frame_cond);
  //pthread_mutex_unlock(&gl_backbuf_mutex);
}
  
//From Hector
void rgbimg(uint8_t *buf, int width, int height)
{
  boost::mutex::scoped_lock lock(backbuf_mutex);
  //int i;
  
  //pthread_mutex_lock(&gl_backbuf_mutex);
  memcpy(gl_rgb_back, buf, width*height*3);
  got_frames++;
  //pthread_cond_signal(&gl_frame_cond);
  cond.notify_one();
  //pthread_mutex_unlock(&gl_backbuf_mutex);
}



class KinectNode
{
public:



  //public:
  ros::NodeHandle n_;
  ros::Timer timer_;
  ros::Timer timer2_;
  tf::TransformBroadcaster tf_;

  ros::Publisher pc_pub_;
  ros::Publisher cam_pub_;

  libusb_device_handle *dev;
  
  double focal_length_x;
  double focal_length_y;
  double pp_x;
  double pp_y;

  double range_fx;
  double range_fy;
  double range_ppx;
  double range_ppy;

  boost::array<double,12> kinect_cam_cal;
  //The range calibration is just a guess until we make a range calibration routine
  boost::array<double,12> kinect_range_cal;

  std::vector<tf::Vector3> rays;

  KinectNode(int argc, char** argv):
    n_("~")
  {
    ROS_INFO("ROS Node Initialized...");
    
    //We calibrated ours, see the included launch file
    n_.param("focal_length_x",focal_length_x,543.112452);
    n_.param("focal_length_y",focal_length_y,546.357325);
    n_.param("principal_x",pp_x,308.036395);
    n_.param("principal_y",pp_y,274.490915);

    //Wild guesses until we make a calibration routine for the range..
    n_.param("range_focal_length_x",range_fx,1.25*543.112451);
    n_.param("range_focal_length_y",range_fy,1.25*546.357);
    n_.param("range_ppx",range_ppx,308.0);
    n_.param("range_ppy",range_ppy,274.0);

    pc_pub_ = n_.advertise<sensor_msgs::PointCloud2>("kinect",1);
    cam_pub_ = n_.advertise<sensor_msgs::Image>("kinect_cam",1);
    
    
    //precalculate the rays
    kinect_cam_cal = MakeCameraCal(focal_length_x, focal_length_y, pp_x, pp_y);
    kinect_range_cal = MakeCameraCal(range_fx,range_fy,range_ppx,range_ppy);
    rays = calc_rays(kinect_range_cal);
    
    test_project_unproject(kinect_cam_cal);

    for (int i=0; i<2048; i++) {
      float v = i/2048.0;
      v = powf(v, 3)* 6;
      t_gamma[i] = v*6*256;
    }
    
    libusb_init(NULL);
    
    dev = libusb_open_device_with_vid_pid(NULL, 0x45e, 0x2ae);
    if (!dev) {
      printf("Could not open device\n");
      exit(1);
    }

    got_frames = 0;
    ROS_INFO("Initializing Kinect Sensor...");
    cams_init(dev, depthimg, rgbimg);
    ROS_INFO("Kinect Sensor Initialized!");

    //kick off a timer callback
    timer_ = n_.createTimer(ros::Duration(0.01),&KinectNode::timerCallback,this);
    
  }

  boost::array<double,12> MakeCameraCal(double fx, double fy, double ppx, double ppy) {
    boost::array<double,12> P;
    P[0] = fx; P[1] = 0.0; P[2] = ppx; P[3] = 0.0;
    P[4] = 0.0; P[5] = fy; P[6] = ppy; P[7] = 0.0;
    P[8] = 0.0; P[9] = 0.0; P[10] = 1.0; P[11] = 0.0;
    
    return P;
  }

  std::vector<tf::Vector3> calc_rays(boost::array<double,12> cal_mat){
    std::vector<tf::Vector3> rays;
    for(int v = 0; v < 480; v++){
      for(int u = 0; u < 640; u++){
	point2 pt(u,v);
	tf::Vector3 ray = unproject_pixel(cal_mat,pt);
	rays.push_back(ray);
      }
    }
    return rays;
  }

  void test_project_unproject(boost::array<double,12> cal_mat){
    tf::Vector3 pt;
    pt.setX(0.0);
    pt.setY(0.0);
    pt.setZ(1.0);
    point2 projected = project_point(cal_mat,pt);
    ROS_INFO("Projects to %f %f",projected.u,projected.v);
    tf::Vector3 unprojected = unproject_pixel(cal_mat,projected);
    ROS_INFO("Unprojects to %f %f %f",unprojected.getX(),unprojected.getY(),unprojected.getZ());
  }

  tf::Vector3 unproject_pixel(const boost::array<double,12>& P,
			      const point2& px){
    tf::Vector3 result;
    result.setX((px.u - P[2]) / P[0]);
    result.setY((px.v - P[6]) / P[5]);
    result.setZ(1);
    return result;
  }

  point2 project_point(const boost::array<double, 12>& P, 
		       const tf::Vector3& xi) {
    tf::Vector3 xo;
    xo.setX(P[0] * xi.x() + P[1] * xi.y() + P[2] * xi.z() + P[3]);
    xo.setY(P[4] * xi.x() + P[5] * xi.y() + P[6] * xi.z() + P[7]);
    xo.setZ(P[8] * xi.x() + P[9] * xi.y() + P[10] * xi.z() + P[11]);
  
    point2 po (xo.x() / xo.z(),
	       xo.y() / xo.z());
    return po;
  }
  
  std::vector<point2> project_cloud(const boost::array<double, 12>& P,
				    const pcl::PointCloud<pcl::PointXYZ>& cloud,
				    const tf::StampedTransform& transform){
    std::vector<point2> image_points;
    for (unsigned int i = 0;i<cloud.points.size();i++) {
      tf::Vector3 ptu_point_tf(cloud.points[i].x, 
			       cloud.points[i].y, 
			       cloud.points[i].z);
      tf::Vector3 cam_point_tf = transform * ptu_point_tf;
      point2 image_pt =  project_point(P, 
				       cam_point_tf);
    image_points.push_back(image_pt);
    }
    return image_points;
    
  }

  void pubPC(){
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    static int seq = 0;
    for(int v = 0; v < 480; v+=2){
      for(int u = 0; u < 640; u+=2){
	
	//Note: this is wrong -- we don't yet know the range scaling function
	//TODO: find out what the actual range scaling function is
	float depth = float(depths[v*640+u])/420.0;
      
	if(depth != 0){       
	  tf::Vector3 ray = rays[v*640+u];
	  tf::Vector3 pt = ray*depth;
	  
	  //account for transform between cameras, in x at least.
	  //TODO: Full extrinsic parameters
	  tf::Vector3 tform_pt = pt;
	  tform_pt.setX(tform_pt.getX()-0.025);
	  
	  point2 proj_pt = project_point(kinect_cam_cal,tform_pt);

	  pcl::PointXYZRGB p;
	  p.x = pt.getX();
	  p.y = -pt.getY();
	  p.z = pt.getZ();
	  unsigned char color[4];

	  int imgv = proj_pt.v;
	  int imgu = proj_pt.u;
	  int cam_idx = imgv*640+imgu;

	  if((0 <= imgu) && (imgu < 640) &&
	     (0 <= imgv) && (imgv < 480)){
	    color[0] = gl_rgb_back[3*cam_idx+2];
	    color[1] = gl_rgb_back[3*cam_idx+1];
	    color[2] = gl_rgb_back[3*cam_idx];
	    color[3] = 0;
	    p.rgb = *((float*)&color[0]);
	  } else {
	    color[0] = 255;
	    color[1] = 255;
	    color[2] = 255;
	    color[3] = 0;
	    p.rgb = *((float*)&color[0]);
	  }
	  
	  cloud.points.push_back(p);
	}
      }
    }
    //ROS_INFO("%d valid points\n",cloud.points.size());

    pcl::toROSMsg(cloud,cloud_msg);
    cloud_msg.header.frame_id = "/kinect_head";
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.seq = seq++;
    pc_pub_.publish(cloud_msg);
  }

  void pubCam(){
    static int cam_seq = 0;
    sensor_msgs::Image cam_img;
    cam_img.encoding = "rgb8";
    cam_img.height = 480;
    cam_img.width = 640;
    cam_img.step = 640 * 3;
    cam_img.data.resize(640*480*3*sizeof(uint8_t));
    memcpy(&(cam_img.data[0]),gl_rgb_back,640*480*3*sizeof(uint8_t));
    cam_img.header.frame_id = "/kinect_head";
    cam_img.header.stamp = ros::Time::now();
    cam_img.header.seq = cam_seq++;
    cam_pub_.publish(cam_img);
  }


  void timerCallback(const ros::TimerEvent& e)
  {
    boost::mutex::scoped_lock lock(backbuf_mutex);
    
    //tf::Transform tilt_tf = tf::Transform(tf::Quaternion(0.0,0.0,0.0,1.0),tf::Point(0.0,0.0,0.0));
    //tf_.sendTransform(tf::StampedTransform(tilt_tf, ros::Time::now(),"kinect_base","kinect_head"));

    static int fcnt = 0;

    while (got_frames < 2) {
      cond.wait(lock);
    }

    memcpy(gl_depth_front, gl_depth_back, sizeof(gl_depth_back));
    memcpy(gl_rgb_front, gl_rgb_back, sizeof(gl_rgb_back));
    
    pubPC();
    pubCam();

    got_frames = 0;

    printf("Frame %d\n", fcnt++);
    
  }

  

};


void libusbThread(){
  ros::Duration(0.01).sleep();
  while(libusb_handle_events(NULL) == 0);
}


int main(int argc, char** argv) 
{
  printf("Kinect camera test\n");
  
  ros::init(argc, argv, "kinect_node");

  KinectNode kn(argc,argv);

  boost::thread spin_thread = boost::thread(boost::bind(&libusbThread));

  ros::spin();
  return 0;
}
