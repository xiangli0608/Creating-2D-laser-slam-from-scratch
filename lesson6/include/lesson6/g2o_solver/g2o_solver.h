#ifndef KARTO_G2OSOLVER_H
#define KARTO_G2OSOLVER_H

#include "open_karto/Mapper.h"
#include "visualization_msgs/MarkerArray.h"
#include "g2o/core/sparse_optimizer.h"

class G2oSolver : public karto::ScanSolver
{
public:
  G2oSolver();
  virtual ~G2oSolver();

public:
  virtual void Clear();
  virtual void Compute();
  virtual const karto::ScanSolver::IdPoseVector &GetCorrections() const;

  virtual void AddNode(karto::Vertex<karto::LocalizedRangeScan> *pVertex);
  virtual void AddConstraint(karto::Edge<karto::LocalizedRangeScan> *pEdge);

  void publishGraphVisualization(visualization_msgs::MarkerArray &marray);

private:
  karto::ScanSolver::IdPoseVector mCorrections;
  g2o::SparseOptimizer mOptimizer;
};

#endif
