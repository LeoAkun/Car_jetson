// Pure offline graph experiment; no ROS or hardware access.
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/inference/Symbol.h>
#include <iostream>
#include <iomanip>
using namespace gtsam;
using namespace gtsam::symbol_shorthand;
int main(int argc,char**argv) {
  int samples=argc>1?atoi(argv[1]):10;
  double dt=argc>2?atof(argv[2]):0.01;
  bool initialize=argc>3?atoi(argv[3]):0;
  double vzsig=argc>4?atof(argv[4]):1e4;
  double yaw=argc>5?atof(argv[5]):0.;
  double roll=argc>6?atof(argv[6]):0.;
  auto p=PreintegrationParams::MakeSharedU(9.80511);
  p->accelerometerCovariance=Matrix33::Identity()*pow(.01,2);
  p->gyroscopeCovariance=Matrix33::Identity()*pow(.001,2);
  p->integrationCovariance=Matrix33::Identity()*pow(1e-4,2);
  imuBias::ConstantBias bias;
  PreintegratedImuMeasurements pim(p,bias);
  ISAM2Params params; params.relinearizeThreshold=.1; params.relinearizeSkip=1;
  ISAM2 optimizer(params);
  Pose3 pose(Rot3::RzRyRx(roll,0,yaw),Point3(0,0,-.11));
  Vector3 vel(0,0,0);
  NavState prevstate;
  if(initialize)prevstate=NavState(pose,vel);
  NonlinearFactorGraph graph;Values values;
  auto priorpose=noiseModel::Diagonal::Sigmas((Vector(6)<<.01,.01,.01,.01,.01,.01).finished());
  auto priorvel=noiseModel::Isotropic::Sigma(3,vzsig);
  auto priorbias=noiseModel::Isotropic::Sigma(6,.001);
  auto corr=noiseModel::Diagonal::Sigmas((Vector(6)<<.05,.05,.05,.1,.1,.1).finished());
  Vector6 bn;bn<<.0002,.0002,.0002,.00001,.00001,.00001;
  try {
    graph.add(PriorFactor<Pose3>(X(0),pose,priorpose));
    graph.add(PriorFactor<Vector3>(V(0),vel,priorvel));
    graph.add(PriorFactor<imuBias::ConstantBias>(B(0),bias,priorbias));
    values.insert(X(0),pose); values.insert(V(0),vel);values.insert(B(0),bias);
    optimizer.update(graph,values);
    graph.resize(0);values.clear();
    std::cout<<"initial_prior_ok ";
    for(int j=0;j<samples;j++)pim.integrateMeasurement(Vector3(0,0,9.80511),Vector3(0,0,0),dt);
    graph.add(ImuFactor(X(0),V(0),X(1),V(1),B(0),pim));
    graph.add(BetweenFactor<imuBias::ConstantBias>(B(0),B(1),bias,noiseModel::Diagonal::Sigmas(sqrt(pim.deltaTij())*bn)));
    graph.add(PriorFactor<Pose3>(X(1),pose,corr));
    auto state=pim.predict(prevstate,bias);
    values.insert(X(1),state.pose());values.insert(V(1),state.v());values.insert(B(1),bias);
    optimizer.update(graph,values); optimizer.update();
    std::cout<<"step1_ok deltaT="<<pim.deltaTij()<<" vel="<<optimizer.calculateEstimate<Vector3>(V(1)).transpose()<<std::endl;
  } catch(const std::exception&e){std::cout<<"FAILED: "<<e.what()<<std::endl;return 2;}
}
