/**
 * @file
 * @brief Loop closing class
 */

#ifndef LOOP_CLOSING_H
#define LOOP_CLOSING_H

#include <ros/ros.h>

#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include "constants.h"
#include "cluster.h"
#include "graph.h"
#include "hash.h"

using namespace std;
using namespace boost;
namespace fs  = filesystem;

namespace slam
{

class Graph;

class LoopClosing
{

public:

  /** \brief Class constructor
   */
  LoopClosing();

  /** \brief Set the graph object
   * \param graph
   */
  inline void setGraph(Graph *graph){graph_ = graph;}

  /** \brief Starts graph
   */
  void run();

  /** \brief Add a cluster to the queue of clusters
   * \param The cluster to be inserted
   */
  void addClusterToQueue(Cluster cluster);

  /** \brief Finalizes the loop closing class
   */
  void finalize();

protected:

  /** \brief Check if there are clusters in the queue
   * @return true if clusters queue is not empty.
   */
  bool checkNewClusterInQueue();

  /** \brief Processes the new cluster
   */
  void processNewCluster();

  /** \brief Searches a loop closing between current cluster and its closest neighbors
   */
  void searchByProximity();

  /** \brief Searches a loop closing between current cluster and all other clusters using the hash
   */
  void searchByHash();

  /** \brief Check if point is in frustum
   * @return true if point is in camera frustum
   * \param 3d world point
   * \param camera pose
   */
  bool isInFrustum(cv::Point3f point, tf::Transform camera_pose);

  /** \brief Remove descriptors and 3D points that are not in the camera frustum
   * \param input descriptors
   * \param input world points
   * \param camera pose where point will be projected
   * \param output descriptors
   * \param output world points
   */
  void filterByFrustum(cv::Mat desc,
                       vector<cv::Point3f> points,
                       tf::Transform camera_pose,
                       cv::Mat& out_desc,
                       vector<cv::Point3f>& out_points);

  /** \brief Tries to close a loop between two clusters
   * @return true if loop closing
   * \param Candidate cluster
   */
  bool closeLoopWithCluster(Cluster candidate);

  /** \brief Get the best candidates to close a loop by hash
   * \param Cluster identifier
   * \param The list of best candidates
   */
  void getCandidates(int cluster_id, vector< pair<int,float> >& candidates);

  /** \brief Read cluster data from file
   * @return The cluster
   * \param Cluster identifier
   */
  Cluster readCluster(int id);

private:

  Cluster c_cluster_; //!> Current cluster to be processed

  list<Cluster> cluster_queue_; //!> Clusters queue to be inserted into the graph

  mutex mutex_cluster_queue_; //!> Mutex for the insertion of new clusters

  Hash hash_; //!> Hash object

  vector< pair<int, vector<float> > > hash_table_;  //!> Hash table: stores a hash for every image. This is the unique variable that grows with the robot trajectory

  vector< pair<int, int > > cluster_lc_found_; //!> Stores all the loop closures (between clusters) found in order to do not repeat them

  int num_loop_closures_; //!> Stores the number of loop closures

  string execution_dir_; //!> Execution directory where all image information will be stored

  string loop_closures_dir_; //!> Directory where images of loop closures will be stored

  Graph* graph_; //!> Graph pointer

  ros::Publisher pub_num_keyframes_; //!> Publishes the number of keyframes

  ros::Publisher pub_num_lc_; //!> Publishes the number of loop closings

  ros::Publisher pub_queue_; //!> Publishes the loop closing queue size

  ros::Publisher pub_lc_matchings_; //!> Publishes the image with the loop closure matchings

  image_geometry::PinholeCameraModel camera_model_; //!> Camera model (left)

};

} // namespace

#endif // LOOP_CLOSING_H