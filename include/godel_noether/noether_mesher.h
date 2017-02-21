#ifndef GODEL_NOETHER_NOETHER_MESHER_H
#define GODEL_NOETHER_NOETHER_MESHER_H

#include <meshing_plugins_base/meshing_base.h>

namespace godel_noether
{

class NoetherMesher : public meshing_plugins_base::MeshingBase
{
public:
  void init(pcl::PointCloud<pcl::PointXYZRGB> input) override;

  bool generateMesh(pcl::PolygonMesh &mesh) override;

private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_;
};

}

#endif // GODEL_NOETHER_NOETHER_MESHER_H
