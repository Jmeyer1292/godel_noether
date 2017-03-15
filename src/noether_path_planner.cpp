#include "godel_noether/noether_path_planner.h"

#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <noether/noether.h>
#include <noether_conversions/noether_conversions.h>
#include <path_sequence_planner/simple_path_sequence_planner.h>
#include <eigen_conversions/eigen_msg.h>

namespace
{

struct PathEndPoints
{
  geometry_msgs::Point a;
  geometry_msgs::Point b;
};

struct SequencePoint
{
  int id;
  bool from_a;
};

std::vector<PathEndPoints> toEndPoints(const std::vector<geometry_msgs::PoseArray>& segments)
{
  std::vector<PathEndPoints> result;
  for (const auto& s : segments)
  {
    const auto& a = s.poses.front().position;
    const auto& b = s.poses.back().position;
    result.push_back({a, b});
  }
  return result;
}

SequencePoint getNextSequencePoint(const geometry_msgs::Point& current, const std::vector<SequencePoint>& so_far,
                                   const std::vector<PathEndPoints>& path_end_points)
{
  SequencePoint min_dist_seq;
  double min_dist = std::numeric_limits<double>::max();

  auto already_in_sequence = [&so_far] (const int idx) {
    for (const auto& s : so_far) {
      if (s.id == idx) return true;
    }
    return false;
  };

  auto pt_dist = [](const geometry_msgs::Point& a, const geometry_msgs::Point& b)
  {
    return std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.z - b.z, 2);
  };

  for (std::size_t i = 0; i < path_end_points.size(); ++i)
  {
    if (!already_in_sequence(i))
    {
      const auto dist_to_a = pt_dist(current, path_end_points[i].a);
      const auto dist_to_b = pt_dist(current, path_end_points[i].b);

      if (dist_to_a < min_dist)
      {
        min_dist = dist_to_a;
        min_dist_seq.id = i;
        min_dist_seq.from_a = true;
      }

      if (dist_to_b < min_dist)
      {
        min_dist = dist_to_b;
        min_dist_seq.id = i;
        min_dist_seq.from_a = false;
      }
    }
  }
  return min_dist_seq;
}

std::vector<geometry_msgs::PoseArray> makeSequence(const std::vector<geometry_msgs::PoseArray>& in,
                                                   const std::vector<SequencePoint>& seqs)
{
  assert(in.size() == seqs.size());
  std::vector<geometry_msgs::PoseArray> rs;
  rs.reserve(in.size());

  for (const auto& seq : seqs)
  {
    rs.push_back(in[seq.id]);
    if (!seq.from_a)
    {
      std::reverse(rs.back().poses.begin(), rs.back().poses.end());
    }
  }
  return rs;
}

std::vector<geometry_msgs::PoseArray> sequence(const std::vector<geometry_msgs::PoseArray>& input)
{
  auto end_points = toEndPoints(input);

  auto get_current_pt = [&end_points] (const SequencePoint& p) {
    if (p.from_a)
      return end_points[p.id].b;
    else
      return end_points[p.id].a;
  };

  std::vector<SequencePoint> sequence;
  sequence.reserve(input.size());

  // Always start with the first point in the first sequence
  sequence.push_back({0, true});

  for (std::size_t i = 1; i < input.size(); ++i)
  {
    // Get the current 'point'
    const auto& pt = get_current_pt(sequence.back());

    // Pick the minimum and add it to the sequence
    const auto next_sequence_pt = getNextSequencePoint(pt, sequence, end_points);
    sequence.push_back(next_sequence_pt);
  }

  assert(sequence.size() == input.size());
  return makeSequence(input, sequence);
}

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
planPaths(vtkSmartPointer<vtkPolyData> mesh,
          const tool_path_planner::ProcessTool& tool)
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

bool godel_noether::NoetherPathPlanner::generatePath(
    std::vector<geometry_msgs::PoseArray> &path)
{
  ROS_INFO("Starting Noether path planning...");
  auto vtk_data = vtkSmartPointer<vtkPolyData>::New();
  vtk_viewer::pclEncodeMeshAndNormals(mesh_, vtk_data);
  vtk_viewer::generateNormals(vtk_data);
  ROS_INFO("generatePath: converted mesh to VTK");

  auto tool = loadTool();
  auto process_paths = planPaths(vtk_data, tool);
  ROS_INFO("generatePath: finished planning paths");

//  //Sequence Paths
//  path_sequence_planner::SimplePathSequencePlanner sequencer;
//  sequencer.setPaths(process_paths);
//  sequencer.linkPaths();

//  // Convert to ROS pose array types
//  path = noether::convertVTKtoGeometryMsgs(sequencer.getPaths());
  auto paths = noether::convertVTKtoGeometryMsgs(process_paths);
  path = sequence(paths);

  ROS_INFO("generatePath: converted to ROS messages - DONE!");

  return true;
}

PLUGINLIB_EXPORT_CLASS(godel_noether::NoetherPathPlanner, path_planning_plugins_base::PathPlanningBase)
