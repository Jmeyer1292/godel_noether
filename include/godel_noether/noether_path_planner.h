#ifndef GODEL_NOETHER_PATH_PLANNER_H
#define GODEL_NOETHER_PATH_PLANNER_H

#include <path_planning_plugins_base/path_planning_base.h>
#include <noether/noether.h>

namespace godel_noether
{

class NoetherPathPlanner : public path_planning_plugins_base::PathPlanningBase
{
public:
  void init(pcl::PolygonMesh mesh) override;
  bool generatePath(std::vector<geometry_msgs::PoseArray> &path) override;
  virtual tool_path_planner::ProcessTool loadTool() const;

private:
  pcl::PolygonMesh mesh_;
};

class NoetherBlendPathPlanner : public NoetherPathPlanner
{
public:
  virtual tool_path_planner::ProcessTool loadTool() const override;
};

class NoetherScanPathPlanner : public NoetherPathPlanner
{
public:
  virtual tool_path_planner::ProcessTool loadTool() const override;
};


}

#endif // GODEL_NOETHER_PATH_PLANNER_H
