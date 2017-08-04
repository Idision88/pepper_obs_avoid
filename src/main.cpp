#include <ros/ros.h>

#include "pepper_obs_avoid.h"



int main( int argc, char** argv )
{
  ros::init( argc, argv, "pbvs_arm" );

  ros::NodeHandle n(std::string("~"));

  pepper_obs_avoid *node = new pepper_obs_avoid(n);

  node->spin();

  delete node;

  return 0;
}




