#include "godel_noether/noether_path_planner.h"

#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <noether/noether.h>
#include <noether_conversions/noether_conversions.h>

namespace
{

tool_path_planner::ProcessTool loadTool()
{
  tool_path_planner::ProcessTool tool;
  tool.pt_spacing = 0.01;
  tool.line_spacing = 0.025;
  tool.tool_offset = 0.0; // currently unused
  tool.intersecting_plane_height = 0.05; // 0.5 works best, not sure if this should be included in the tool
  tool.nearest_neighbors = 5; // not sure if this should be a part of the tool
  tool.min_hole_size = 0.01;
  return tool;
}

std::vector<tool_path_planner::ProcessPath>
planPaths(vtkSmartPointer<vtkPolyData> mesh, const tool_path_planner::ProcessTool& tool)
{
  std::vector<vtkSmartPointer<vtkPolyData>> meshes;
  meshes.push_back(mesh);

  tool_path_planner::RasterToolPathPlanner planner;
  planner.setTool(tool);
  std::vector<std::vector<tool_path_planner::ProcessPath>> paths;
  planner.planPaths(meshes, paths);

  assert(paths.size() == 1);
  return paths.front();
}

} // anon namespace

void godel_noether::NoetherPathPlanner::init(pcl::PolygonMesh mesh)
{
  mesh_ = mesh;
}

bool godel_noether::NoetherPathPlanner::generatePath(std::vector<geometry_msgs::PoseArray> &path)
{
  ROS_INFO("Starting Noether path planning...");
  auto vtk_data = vtkSmartPointer<vtkPolyData>::New();
  vtk_viewer::pclEncodeMeshAndNormals(mesh_, vtk_data);
  vtk_viewer::generateNormals(vtk_data);
  ROS_INFO("generatePath: converted mesh to VTK");

  auto tool = loadTool();
  auto process_paths = planPaths(vtk_data, tool);
  ROS_INFO("generatePath: finished planning paths");

  // Convert to ROS pose array types
  path = posesConvertVTKtoGeometryMsgs(process_paths);
  ROS_INFO("generatePath: converted to ROS messages - DONE!");

  return true;
}

PLUGINLIB_EXPORT_CLASS(godel_noether::NoetherPathPlanner, path_planning_plugins_base::PathPlanningBase)
