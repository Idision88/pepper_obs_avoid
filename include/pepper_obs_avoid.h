#include <ros/ros.h>
#include <boost/thread.hpp>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMatrix.h>
#include <visp/vpVelocityTwistMatrix.h>
#include <visp/vpXmlParserCamera.h>
#include <visp/vpXmlParserHomogeneousMatrix.h>

#include <visp_bridge/3dpose.h>
#include <visp_bridge/image.h>
#include <visp_bridge/camera.h>

#include <visp_naoqi/vpNaoqiRobot.h>
#include <visp_naoqi/common/vpPepperFollowPeople.h>

#include <qi/log.hpp>
#include <qi/applicationsession.hpp>
#include <qi/anyobject.hpp>

class pepper_obs_avoid
{
public:
  
  pepper_obs_avoid(ros::NodeHandle &nh);
  ~pepper_obs_avoid();
  void spin();
  
protected:
  
  void getCameraInfoCb(const sensor_msgs::CameraInfoConstPtr &msg);
  void getLaserCb(const sensor_msgs::LaserScanConstPtr &msg);
  std::vector<double> obs_avoidance_control(double vx, double vy, double wz);

  boost::mutex lock_;
  std::string m_robotIp;

  //Robot
  vpNaoqiRobot * robot;
  qi::SessionPtr m_session;
  std::vector<std::string> m_jointNames_head;

  // Servo Follow people
  vpPepperFollowPeople * m_follow_people;

  // Control
  //std::vector<float> m_v_des;
  vpMatrix m_eJe;
  bool m_servo_enabled;
  
  //Visp
  vpCameraParameters m_cam;
  vpMatrix m_L;
  std::vector <vpMatrix> m_A;
  
  vpHomogeneousMatrix m_cMh;
  vpImage<unsigned char> I;
  
  // ROS
  ros::NodeHandle m_n;
  std::string m_cameraInfoTopicName;
  std::string m_laserTopicName;
  ros::Subscriber m_cameraInfoSub;
  ros::Subscriber m_laserSub;
  
  //Laser
  std::vector<float> m_laser_data;
  bool m_laserInit;
  float m_angleMin;
  float m_angleMax;
  float m_angleIncrement;
  float m_min_dist;
  
  std::vector< vpHomogeneousMatrix> m_Hlaser;
  std::vector< vpMatrix> m_Vlaser;
  
  int freq;
  
  //conditions
  bool m_camInfoIsInitialized;
  bool m_node_init;
  
};
