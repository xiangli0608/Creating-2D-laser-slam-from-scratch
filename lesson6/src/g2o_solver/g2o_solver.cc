#include "lesson6/g2o_solver/g2o_solver.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/types/slam2d/types_slam2d.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include <ros/console.h>

typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>> SlamBlockSolver;
typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

G2oSolver::G2oSolver()
{
  // 第1步：创建一个线性求解器LinearSolver
  SlamLinearSolver *linearSolver = new SlamLinearSolver();
  linearSolver->setBlockOrdering(false);
  // 第2步：创建BlockSolver。并用上面定义的线性求解器初始化
  SlamBlockSolver *blockSolver = new SlamBlockSolver(linearSolver);
  // 第3步：创建总求解器solver。并从GN, LM, DogLeg 中选一个，再用上述块求解器BlockSolver初始化
  g2o::OptimizationAlgorithmLevenberg *solver =
      new g2o::OptimizationAlgorithmLevenberg(blockSolver);
  // 第4步：创建稀疏优化器（SparseOptimizer）
  mOptimizer.setAlgorithm(solver);
}

G2oSolver::~G2oSolver()
{
  // freeing the graph memory
  mOptimizer.clear();
}

void G2oSolver::Clear()
{
  // freeing the graph memory
  ROS_INFO("[g2o] Clearing Optimizer...");
  mCorrections.clear();
}

void G2oSolver::AddNode(karto::Vertex<karto::LocalizedRangeScan> *pVertex)
{
  karto::Pose2 odom = pVertex->GetObject()->GetCorrectedPose();
  g2o::VertexSE2 *poseVertex = new g2o::VertexSE2;
  poseVertex->setEstimate(g2o::SE2(odom.GetX(), odom.GetY(), odom.GetHeading()));
  poseVertex->setId(pVertex->GetObject()->GetUniqueId());
  mOptimizer.addVertex(poseVertex);
  ROS_DEBUG("[g2o] Adding node %d.", pVertex->GetObject()->GetUniqueId());
}

void G2oSolver::AddConstraint(karto::Edge<karto::LocalizedRangeScan> *pEdge)
{
  // Create a new edge
  g2o::EdgeSE2 *odometry = new g2o::EdgeSE2;

  // Set source and target
  int sourceID = pEdge->GetSource()->GetObject()->GetUniqueId();
  int targetID = pEdge->GetTarget()->GetObject()->GetUniqueId();
  odometry->vertices()[0] = mOptimizer.vertex(sourceID);
  odometry->vertices()[1] = mOptimizer.vertex(targetID);
  if (odometry->vertices()[0] == NULL)
  {
    ROS_ERROR("[g2o] Source vertex with id %d does not exist!", sourceID);
    delete odometry;
    return;
  }
  if (odometry->vertices()[1] == NULL)
  {
    ROS_ERROR("[g2o] Target vertex with id %d does not exist!", targetID);
    delete odometry;
    return;
  }

  // Set the measurement (odometry distance between vertices)
  karto::LinkInfo *pLinkInfo = (karto::LinkInfo *)(pEdge->GetLabel());
  karto::Pose2 diff = pLinkInfo->GetPoseDifference();
  g2o::SE2 measurement(diff.GetX(), diff.GetY(), diff.GetHeading());
  odometry->setMeasurement(measurement);

  // Set the covariance of the measurement
  karto::Matrix3 precisionMatrix = pLinkInfo->GetCovariance().Inverse();
  Eigen::Matrix<double, 3, 3> info;
  info(0, 0) = precisionMatrix(0, 0);
  info(0, 1) = info(1, 0) = precisionMatrix(0, 1);
  info(0, 2) = info(2, 0) = precisionMatrix(0, 2);
  info(1, 1) = precisionMatrix(1, 1);
  info(1, 2) = info(2, 1) = precisionMatrix(1, 2);
  info(2, 2) = precisionMatrix(2, 2);
  odometry->setInformation(info);

  // Add the constraint to the optimizer
  ROS_DEBUG("[g2o] Adding Edge from node %d to node %d.", sourceID, targetID);
  mOptimizer.addEdge(odometry);
}

void G2oSolver::Compute()
{
  mCorrections.clear();

  // Fix the first node in the graph to hold the map in place
  g2o::OptimizableGraph::Vertex *first = mOptimizer.vertex(0);
  if (!first)
  {
    ROS_ERROR("[g2o] No Node with ID 0 found!");
    return;
  }
  first->setFixed(true);

  // Do the graph optimization
  mOptimizer.initializeOptimization();
  int iter = mOptimizer.optimize(40);
  if (iter > 0)
  {
    ROS_INFO("[g2o] Optimization finished after %d iterations.", iter);
  }
  else
  {
    ROS_ERROR("[g2o] Optimization failed, result might be invalid!");
    return;
  }

  // Write the result so it can be used by the mapper
  g2o::SparseOptimizer::VertexContainer nodes = mOptimizer.activeVertices();
  for (g2o::SparseOptimizer::VertexContainer::const_iterator n = nodes.begin(); n != nodes.end(); n++)
  {
    double estimate[3];
    if ((*n)->getEstimateData(estimate))
    {
      karto::Pose2 pose(estimate[0], estimate[1], estimate[2]);
      mCorrections.push_back(std::make_pair((*n)->id(), pose));
    }
    else
    {
      ROS_ERROR("[g2o] Could not get estimated pose from Optimizer!");
    }
  }
}

const karto::ScanSolver::IdPoseVector &G2oSolver::GetCorrections() const
{
  return mCorrections;
}

/**
 * Fill out a visualization_msg to display the odometry graph
 * This function also identifies the loop closure edges and adds them to the
 * loop_closure_edges_ data for further processing
 */
void G2oSolver::publishGraphVisualization(visualization_msgs::MarkerArray &marray)
{
  // Vertices are round, red spheres
  visualization_msgs::Marker m;
  m.header.frame_id = "map";
  m.header.stamp = ros::Time::now();
  m.id = 0;
  m.ns = "karto";
  m.type = visualization_msgs::Marker::SPHERE;
  m.pose.position.x = 0.0;
  m.pose.position.y = 0.0;
  m.pose.position.z = 0.0;
  m.scale.x = 0.1;
  m.scale.y = 0.1;
  m.scale.z = 0.1;
  m.color.r = 1.0;
  m.color.g = 0;
  m.color.b = 0.0;
  m.color.a = 1.0;
  m.lifetime = ros::Duration(0);

  // Odometry edges are opaque blue line strips
  visualization_msgs::Marker edge;
  edge.header.frame_id = "map";
  edge.header.stamp = ros::Time::now();
  edge.action = visualization_msgs::Marker::ADD;
  edge.ns = "karto";
  edge.id = 0;
  edge.type = visualization_msgs::Marker::LINE_STRIP;
  edge.scale.x = 0.1;
  edge.scale.y = 0.1;
  edge.scale.z = 0.1;
  edge.color.a = 1.0;
  edge.color.r = 0.0;
  edge.color.g = 0.0;
  edge.color.b = 1.0;

  // Loop edges are purple, opacity depends on backend state
  visualization_msgs::Marker loop_edge;
  loop_edge.header.frame_id = "map";
  loop_edge.header.stamp = ros::Time::now();
  loop_edge.action = visualization_msgs::Marker::ADD;
  loop_edge.ns = "karto";
  loop_edge.id = 0;
  loop_edge.type = visualization_msgs::Marker::LINE_STRIP;
  loop_edge.scale.x = 0.1;
  loop_edge.scale.y = 0.1;
  loop_edge.scale.z = 0.1;
  loop_edge.color.a = 1.0;
  loop_edge.color.r = 1.0;
  loop_edge.color.g = 0.0;
  loop_edge.color.b = 1.0;

  int id = (int)marray.markers.size();
  m.action = visualization_msgs::Marker::ADD;

  std::set<int> vertex_ids;
  // HyperGraph Edges
  for (g2o::SparseOptimizer::EdgeSet::iterator edge_it = mOptimizer.edges().begin();
       edge_it != mOptimizer.edges().end(); ++edge_it)
  {
    // Is it a unary edge? Need to skip or else die a bad death
    if ((*edge_it)->vertices().size() < 2)
      continue;

    int id1, id2;
    g2o::VertexSE2 *v1, *v2;

    v1 = dynamic_cast<g2o::VertexSE2 *>((*edge_it)->vertices()[0]);
    v2 = dynamic_cast<g2o::VertexSE2 *>((*edge_it)->vertices()[1]);

    geometry_msgs::Point p1, p2;
    p1.x = v1->estimate()[0];
    p1.y = v1->estimate()[1];
    p2.x = v2->estimate()[0];
    p2.y = v2->estimate()[1];

    if ((v2->id() - v1->id()) < 70) // not a loop closure
    {
      edge.points.clear();
      edge.points.push_back(p1);
      edge.points.push_back(p2);
      edge.id = id;
      marray.markers.push_back(visualization_msgs::Marker(edge));
      id++;
    }
    else // it's a loop closure
    {
      loop_edge.points.clear();
      loop_edge.points.push_back(p1);
      loop_edge.points.push_back(p2);
      loop_edge.id = id++;

      loop_edge.color.a = 1.0;

      marray.markers.push_back(visualization_msgs::Marker(loop_edge));
    }

    // Check the vertices exist, if not add
    if (vertex_ids.find(v2->id()) == vertex_ids.end())
    {
      // Add the vertex to the marker array
      m.id = id;
      m.pose.position.x = p2.x;
      m.pose.position.y = p2.y;
      vertex_ids.insert(id2);
      marray.markers.push_back(visualization_msgs::Marker(m));
      id++;
    }
  }
}
