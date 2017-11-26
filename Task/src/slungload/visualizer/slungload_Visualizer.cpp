
#include "slungload/visualizer/slungload_Visualizer.hpp"

namespace rai {
namespace Vis {

slungload_Visualizer::slungload_Visualizer() :
    graphics(600, 450),
    quadrotor(0.3),
    load(0.03),
    Target(0.055),
    tether1(0.01, 1.0),
    tether2(0.001, 0.3),
    tether3(0.001, 0.3),
    background("sky"){

  Target.setColor({1.0, 0.0, 0.0});
  load.setColor({0.0, 1.0, 0.0});

  defaultPose_.setIdentity();
  rai::Math::MathFunc::rotateHTabout_x_axis(defaultPose_, -M_PI_2);

  graphics.addSuperObject(&quadrotor);
  graphics.addObject(&Target);
  graphics.addObject(&load);
  graphics.addObject(&tether1);
  //graphics.addObject(&tether2);
  //graphics.addObject(&tether3);
  graphics.addBackground(&background);
  //graphics.setBackgroundColor(1, 1, 1, 1);

  Eigen::Vector3d relPos;
  relPos << -3, 0, 0;
  std::vector<float> pos = {-100, 0, 0}, spec = {0.7, 0.7, 0.7}, amb = {0.7, 0.7, 0.7}, diff = {0.7, 0.7, 0.7};

  rai_graphics::LightProp lprop;
  lprop.amb_light = amb;
  lprop.spec_light = spec;
  lprop.diff_light = diff;
  lprop.pos_light = pos;
  rai_graphics::CameraProp cprop;
  cprop.toFollow = quadrotor.basePtr();
  cprop.relativeDist = relPos;

  graphics.setCameraProp(cprop);
  graphics.setLightProp(lprop);
  graphics.start();

}

slungload_Visualizer::~slungload_Visualizer() {
  graphics.end();
}

void slungload_Visualizer::drawWorld(HomogeneousTransform &bodyPose, Position &quadPos, Quaternion &quadAtt, Position &loadPos) {
  Eigen::Vector3d pos;
  Eigen::Vector3d end;
  HomogeneousTransform quadPose, tetherPose;
  Quaternion tetherAtt;
  RotationMatrix rotmat;
  Position zAxis, loadDir;
  zAxis << 0.0, 0.0, 1.0;

  rotmat = bodyPose.topLeftCorner(3, 3);
  pos = bodyPose.topRightCorner(3, 1);

  quadPose.setIdentity();
  quadPose.topRightCorner(3, 1) = quadPos;
  quadPose.topLeftCorner(3,3) = rai::Math::MathFunc::quatToRotMat(quadAtt);
  end = rotmat * end;
  quadPose = quadPose * defaultPose_;

  quadrotor.setPose(quadPose);
  quadrotor.spinRotors();

  load.setPos(loadPos);

  tetherPose.setIdentity();
  loadDir = (quadPos - loadPos) / (quadPos - loadPos).norm();

  // by dh
  Position xAxis, yAxis;
  RotationMatrix tetherRotmat;

  zAxis = loadDir;
  if (zAxis(0) == 0.0 && zAxis(1) == 0.0)
    xAxis << 1.0, 0.0, 0.0;
  else
    xAxis << zAxis(1), -zAxis(0), 0;
  yAxis = zAxis.cross(xAxis);

  tetherRotmat << xAxis(0), yAxis(0), zAxis(0),
                  xAxis(1), yAxis(1), zAxis(1),
                  xAxis(2), yAxis(2), zAxis(2);

  tetherAtt(0) = std::sqrt((1 + loadDir.dot(zAxis))/2);
  tetherAtt.segment<3>(1) = std::sqrt(1 - tetherAtt(0)*tetherAtt(0)) * zAxis.cross(loadDir);
  tetherAtt.normalize();

  tetherPose.topLeftCorner(3,3) = tetherRotmat;
  tetherPose.topRightCorner(3,1) = quadPos + 1.0 * ( loadPos - quadPos );

  tether1.setPose(tetherPose);
  Target.setPos(end);

}

rai_graphics::RAI_graphics *slungload_Visualizer::getGraphics() {
  return &graphics;
}

}
}