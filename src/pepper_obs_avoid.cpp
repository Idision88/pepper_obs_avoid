#include <iostream>
#include <vector>
#include <algorithm>
#include <map>

#include <visp3/core/vpPoint.h>
#include <visp/vpTranslationVector.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpXmlParserHomogeneousMatrix.h>
#include <visp/vpCalibration.h>

#include <QuadProg++.hh>

#include "pepper_obs_avoid.h"

#include <visp3/gui/vpDisplayX.h>



vpDisplayX d;
pepper_obs_avoid::pepper_obs_avoid(ros::NodeHandle &nh):
  lock_(), m_cam(), m_camInfoIsInitialized(false), m_laser_data(0), m_laserInit(false), m_angleMin(0.0), m_angleMax(0.0), m_L(1,6),
  m_min_dist(0.7), m_A(), m_v_des(2), robot()
{
  // read in config options
  m_n = nh;
  m_n.param( "frequency", freq, 30);
  m_n.param<std::string>("cameraInfoName", m_cameraInfoTopicName, "/camera/camera_info");
  m_n.param<std::string>("laserTopicName", m_laserTopicName, "/pepper_robot/laser");
  m_n.param<std::string>("robotIp", m_robotIp, "192.168.0.24");

  m_cameraInfoSub = m_n.subscribe( m_cameraInfoTopicName, 1, (boost::function < void(const sensor_msgs::CameraInfoConstPtr & )>) boost::bind( &pepper_obs_avoid::getCameraInfoCb, this, _1 ));
  m_laserSub = m_n.subscribe( m_laserTopicName, 1, (boost::function < void(const sensor_msgs::LaserScanConstPtr & )>) boost::bind( &pepper_obs_avoid::getLaserCb, this, _1 ));

  // Init viewer
  I.resize(480, 640, 0);
  d.init(I);
  vpDisplay::setTitle(I, "ViSP viewer");

  m_eJe.resize(6, 2);
  m_eJe = 0;
  m_eJe[0][0] = 1;  // vx
  m_eJe[5][1] = 1;  // wz

  //Interaction matrix
  m_L[0][0] = -1;

  m_node_init = true;

  //Desired velocity
  m_v_des[0] = 0.1;
  m_v_des[1] = 0.0001;


  // Create a session to connect with the Robot
  m_session = qi::makeSession();
  std::string ip_port = "tcp://" + m_robotIp + ":9559";
  m_session->connect(ip_port);

  robot = new vpNaoqiRobot(m_session);
  robot->open();

  ROS_INFO("Launch pepper_obs_avoid node");
}

pepper_obs_avoid::~pepper_obs_avoid(){

}

void pepper_obs_avoid::spin()
{
  ros::Rate loop_rate(freq);
  while(ros::ok()){

    vpMouseButton::vpMouseButtonType button;
    bool ret = vpDisplay::getClick(I, button, false);
    vpDisplay::displayText(I, 30, 30, "Press right button to exit", vpColor::green);

    if (m_laserInit)
    {
      //std::cout << "computing" << std::endl;
      obs_avoidance_control();
    }

    vpDisplay::flush(I);

    if (ret && button == vpMouseButton::button3)
    {
      robot->stopBase();
      break;
    }
    ret = false;
    ros::spinOnce();
    loop_rate.sleep();
  }

  robot->stopBase();
}

void pepper_obs_avoid::obs_avoidance_control()
{
  Matrix<double> G, CE, CI;
  Vector<double> g0, ce0, ci0, x;
  int n, m, p;
  char ch;

  n = 2;

  G.resize(n, n);
  G[0][0] = std::abs(m_v_des[0]);
  G[0][1] = 0.;
  G[1][1] = std::abs(m_v_des[1]);
  G[1][0] = 0.;

  g0.resize(n);
  g0[0] = - G[0][0] * m_v_des[0] ;
  g0[1] = - G[1][1] * m_v_des[1] ;

  double lambda  = 1;
  CI.resize(n, m_A.size());
  ci0.resize(m_A.size());
  for (unsigned int i = 0; i < m_A.size(); i++)
  {
    CI[0][i] = m_A[i][0][0];
    CI[1][i] = m_A[i][0][1];
    ci0[i] = lambda * (m_laser_data[i] - m_min_dist);
  }

  //std::cout << m_laser_data[0] << std::endl << m_laser_data[4]  << std::endl;

  m = 1;
  CE.resize(n, m);
  {
    std::istringstream is("0.00000000000001, "
                          "0.00000000000001");

    for (int i = 0; i < n; i++)
      for (int j = 0; j < m; j++)
        is >> CE[i][j] >> ch;
  }

  ce0.resize(m);
  {
    std::istringstream is("0.00000000001 ");

    for (int j = 0; j < m; j++)
      is >> ce0[j] >> ch;
  }
  //  std::cout << "G: " << std::endl  << G <<  std::endl;

  // std::cout << "g0: " << std::endl << g0 << std::endl;
  //std::cout << "-------------" << std::endl;
  //std::cout << "CE: " << std::endl << CE << std::endl;
  //std::cout << "ce0: " << std::endl << ce0 << std::endl;
  //std::cout << "-------------" << std::endl;
  //std::cout << "CI: " << std::endl << CI << std::endl;
  // std::cout << "ci0: " << std::endl << ci0 << std::endl;
  //std::cout << "-------------" << std::endl;
  x.resize(n);
  std::cout << "f: " << solve_quadprog(G, g0, CE, ce0, CI, ci0, x) << std::endl;
  std::cout << "x: " << x << std::endl;

  robot->setBaseVelocity(x[0],0.0,x[1]);

}

void pepper_obs_avoid::getCameraInfoCb(const sensor_msgs::CameraInfoConstPtr &msg)
{
  std::cout << "Received Camera INFO"<<std::endl;
  // Convert the paramenter in the visp format
  m_cam = visp_bridge::toVispCameraParameters(*msg);
  m_cam.printParameters();

  // Stop the subscriber (we don't need it anymore)
  this->m_cameraInfoSub.shutdown();

  m_camInfoIsInitialized = 1;
}

void  pepper_obs_avoid::getLaserCb(const sensor_msgs::LaserScanConstPtr &msg)
{
  m_laser_data.clear();
  for (unsigned int i = 0; i < msg->ranges.size(); i++)
  {
    if (i > 22 && i < 38)//msg->ranges[i] >= 0)
    {
      m_laser_data.push_back(msg->ranges[i]);
      //std::cout << "copy num" << i << "equal to" << msg->ranges[i] <<   std::endl;

    }
  }

  if (!m_laserInit)
  {
    m_angleMin = msg->angle_min;
    m_angleMax = msg->angle_max;
    m_angleIncrement = msg->angle_increment;
    m_laserInit = true;

    // Initialize transformations matrix

    for (unsigned int i = 0; i < msg->ranges.size(); i++)
    {
      vpRxyzVector rxyz;
      rxyz[2] = m_angleMin +  i * m_angleIncrement; // psi angle in rad around z axis
      vpRotationMatrix R(rxyz);
      vpTranslationVector t(0.0,0.0,0.0);
      vpHomogeneousMatrix MtempR(t,R);
      vpHomogeneousMatrix Mtempt(0.0,0.0,0.0,0.0,0.0,0.0);
      vpVelocityTwistMatrix V(MtempR*Mtempt);
      //      vpMatrix v_(6,2);
      //      for (unsigned int j = 0; j < 6; j++)
      //      {
      //        v_[j][0] = V[j][0];
      //        v_[j][1] = V[j][5];
      //      }

      //      std::cout << V << std::endl;
      //      std::cout << "-- " << std::endl;
      //      std::cout<< "rxyz" <<  std::endl << rxyz << std::endl;
      //      std::cout<< "MtempR" << std::endl  << MtempR << std::endl;
      //      std::cout<< "V" <<  std::endl << V << std::endl;
      //      std::cout<< "m_eJe" <<  std::endl << m_eJe << std::endl;
      //      std::cout<< "m_L" <<  std::endl << m_L << std::endl;
      //      std::cout<< "V * m_eJe" <<  std::endl << V * m_eJe << std::endl;
      //      std::cout<< "(m_L * V * m_eJe" <<  std::endl << m_L * V * m_eJe << std::endl;

      if (i > 22 && i < 38)//msg->ranges[i] >= 0)
      {
        m_A.push_back(m_L * V * m_eJe);
        //std::cout << V * m_eJe << std::endl;
        // std::cout << "-------------- " << std::endl;
      }
    }

    //        for (unsigned int i = 0; i < m_A.size(); i++)
    //        {
    //          std::cout << "m_A: " << m_A[i] << std::endl;
    //        }

  }

  //  for (unsigned int i = 0; i < m_laser_data.size(); i++)
  //  {
  //    std::cout << "m_laser_data: " <<m_laser_data[i] << std::endl;
  //  }
  //  std::cout <<" m_laser_data.size()" << m_laser_data.size() << std::endl;
  //  std::cout <<" m_A.size() " << m_A.size() << std::endl;

}
