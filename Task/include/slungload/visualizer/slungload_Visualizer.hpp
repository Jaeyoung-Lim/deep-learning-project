
#include "raiCommon/math/RAI_math.hpp"
#include "raiGraphics/RAI_graphics.hpp"
#include "raiGraphics/obj/Mesh.hpp"
#include "raiGraphics/obj/Cylinder.hpp"
#include "raiGraphics/obj/Sphere.hpp"
#include "raiGraphics/obj/Quadrotor.hpp"



namespace rai {
namespace Vis {

class slungload_Visualizer {

 public:
  using GeneralizedCoordinate = Eigen::Matrix<double, 10, 1>;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  slungload_Visualizer();

  ~slungload_Visualizer();

  void setTerrain(std::string fileName);
  void drawWorld(HomogeneousTransform &visualizationPose, rai::Position &quadPos, rai::Quaternion &quadAtt, rai::Position &loadPos);
  rai_graphics::RAI_graphics* getGraphics();

private:
  rai_graphics::RAI_graphics graphics;
  rai_graphics::object::Quadrotor quadrotor;
  rai_graphics::object::Sphere Target;
  rai_graphics::object::Sphere load;
  rai_graphics::object::Cylinder tether1;
  rai_graphics::object::Cylinder tether2;
  rai_graphics::object::Cylinder tether3;
  rai_graphics::object::Background background;

  HomogeneousTransform defaultPose_;
};

}
}
