#include <godel_noether/noether_mesher.h>
#include <pluginlib/class_list_macros.h>
#include <vtk_viewer/vtk_utils.h>
#include <boost/make_shared.hpp>
#include <pcl/common/io.h>
#include <ros/console.h>

void godel_noether::NoetherMesher::init(pcl::PointCloud<pcl::PointXYZRGB> input)
{
  ROS_INFO("Initializing godel_noether::NoetherMesher");
  input_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::copyPointCloud(input, *input_);
}

bool godel_noether::NoetherMesher::generateMesh(pcl::PolygonMesh &mesh)
{
  ROS_INFO("godel_noether::NoetherMesher::generateMesh");
  auto normals = vtk_viewer::pclEstimateNormals(input_);
  auto pcl_mesh = vtk_viewer::pclGridProjectionMesh(normals);
  mesh = pcl_mesh;
  return true;
}

PLUGINLIB_EXPORT_CLASS(godel_noether::NoetherMesher, meshing_plugins_base::MeshingBase)
