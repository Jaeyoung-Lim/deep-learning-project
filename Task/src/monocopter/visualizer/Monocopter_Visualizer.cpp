
#include "monocopter/visualizer/Monocopter_Visualizer.hpp"

namespace rai {
namespace Vis {

Monocopter_Visualizer::Monocopter_Visualizer() :
    graphics(600, 450),
    quadrotor(0.3),
    Target(0.055),
    background("sky"){

  Target.setColor({1.0, 0.0, 0.0});

  defaultPose_.setIdentity();
  rai::Math::MathFunc::rotateHTabout_x_axis(defaultPose_, -M_PI_2);

  graphics.addSuperObject(&quadrotor);
  graphics.addObject(&Target);
  graphics.addBackground(&background);
  //graphics.setBackgroundColor(1, 1, 1, 1);

  Eigen::Vector3d relPos, targetPos;
  relPos << -3, 0, 0;
  targetPos << 0.0, 0.0, 0.0;
  Target.setPos(targetPos);
  std::vector<float> pos = {-100, 0, 0}, spec = {0.7, 0.7, 0.7}, amb = {0.7, 0.7, 0.7}, diff = {0.7, 0.7, 0.7};

  rai_graphics::LightProp lprop;
  lprop.amb_light = amb;
  lprop.spec_light = spec;
  lprop.diff_light = diff;
  lprop.pos_light = pos;
  rai_graphics::CameraProp cprop;
  cprop.toFollow = &Target;
  cprop.relativeDist = relPos;

  graphics.setCameraProp(cprop);
  graphics.setLightProp(lprop);
  graphics.start();

}

Monocopter_Visualizer::~Monocopter_Visualizer() {
  graphics.end();
}

void Monocopter_Visualizer::drawWorld(HomogeneousTransform &bodyPose, Position &quadPos, Quaternion &quadAtt) {
  Eigen::Vector3d pos;
  HomogeneousTransform quadPose;
  RotationMatrix rotmat;

  rotmat = bodyPose.topLeftCorner(3, 3);
  pos = bodyPose.topRightCorner(3, 1);

  quadPose.setIdentity();
  quadPose.topRightCorner(3, 1) = quadPos;
  quadPose.topLeftCorner(3,3) = rai::Math::MathFunc::quatToRotMat(quadAtt);

  quadPose = quadPose * defaultPose_;

  quadrotor.setPose(quadPose);
  quadrotor.spinRotors();

}

rai_graphics::RAI_graphics *Monocopter_Visualizer::getGraphics() {
  return &graphics;
}
}
}