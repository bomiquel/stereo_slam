#include <ros/ros.h>

#include <boost/thread.hpp>
#include <boost/filesystem.hpp>

#include "localization/constants.h"
#include "localization/frame_publisher.h"
#include "localization/tracking.h"
#include "localization/graph.h"
#include "localization/loop_closing.h"

namespace fs  = boost::filesystem;

/** \brief Read the node parameters
  */
void readParameters(slam::Tracking::Params &tracking_params)
{
  ros::NodeHandle nhp("~");
  string odom_topic, camera_topic;
  nhp.param("odom_topic",   tracking_params.odom_topic,   string(""));
  nhp.param("camera_topic", tracking_params.camera_topic, string(""));
}

/** \brief Main entry point
  */
int main(int argc, char **argv)
{
  // Override SIGINT handler
  ros::init(argc, argv, "stereo_slam");
  ros::start();

  // Execution directory
  if (fs::is_directory(slam::WORKING_DIRECTORY))
    fs::remove_all(slam::WORKING_DIRECTORY);
  fs::path dir(slam::WORKING_DIRECTORY);
  if (!fs::create_directory(dir))
    ROS_ERROR("[Localization:] ERROR -> Impossible to create the execution directory.");

  // For debugging purposes
  slam::FramePublisher frame_publisher;

  // Threads
  slam::LoopClosing loop_closing;
  slam::Graph graph(&loop_closing);
  slam::Tracking tracker(&frame_publisher, &graph);

  // Read parameters
  slam::Tracking::Params tracking_params;
  readParameters(tracking_params);

  // Set the parameters for every object
  tracker.setParams(tracking_params);
  loop_closing.setGraph(&graph);

  // Launch threads
  boost::thread trackingThread(&slam::Tracking::run, &tracker);
  boost::thread graphThread(&slam::Graph::run, &graph);
  boost::thread loopClosingThread(&slam::LoopClosing::run, &loop_closing);

  // ROS spin
  ros::Rate r(10);
  while (ros::ok())
  {
    r.sleep();
  }

  // Loop closing object is the only one that needs finalization
  loop_closing.finalize();

  ros::shutdown();

  return 0;
}