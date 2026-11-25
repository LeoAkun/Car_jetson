// Offline-only exact first-step measurements; no ROS initialization or publish.
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/inference/Symbol.h>
#include <fstream>
#include <iostream>
#include <iomanip>
using namespace gtsam;using namespace gtsam::symbol_shorthand;
int main(int argc,char**argv){
 std::ifstream f(argv[1]); int init=argc>2?atoi(argv[2]):0;double vsig=argc>3?atof(argv[3]):1e4;bool qr=argc>4?atoi(argv[4]):false;
 double a[14];for(auto &v:a)f>>v; int count;f>>count;
 auto getpose=[&](int off){float x=a[off],y=a[off+1],z=a[off+2],qx=a[off+3],qy=a[off+4],qz=a[off+5],qw=a[off+6];return Pose3(Rot3::Quaternion(qw,qx,qy,qz),Point3(x,y,z)).compose(Pose3(Rot3(),Point3(0,0,-.11)));};
 Pose3 p0=getpose(0),p1=getpose(7);Vector3 v0=Vector3::Zero();imuBias::ConstantBias b0;
 auto p=PreintegrationParams::MakeSharedU((float)9.80511);p->accelerometerCovariance=Matrix33::Identity()*pow((float).01,2);p->gyroscopeCovariance=Matrix33::Identity()*pow((float).001,2);p->integrationCovariance=Matrix33::Identity()*pow(1e-4,2);PreintegratedImuMeasurements pim(p,b0);
 ISAM2Params ip;ip.relinearizeThreshold=.1;ip.relinearizeSkip=1;if(qr)ip.factorization=ISAM2Params::QR;ISAM2 opt(ip);
 auto poseN=noiseModel::Diagonal::Sigmas((Vector(6)<<.01,.01,.01,.01,.01,.01).finished());auto velN=noiseModel::Isotropic::Sigma(3,vsig);auto biasN=noiseModel::Isotropic::Sigma(6,.001);auto corrN=noiseModel::Diagonal::Sigmas((Vector(6)<<.05,.05,.05,.1,.1,.1).finished());Vector6 bn;bn<<(float).0002,(float).0002,(float).0002,(float).00001,(float).00001,(float).00001;
 NonlinearFactorGraph g;Values v;NavState state;if(init)state=NavState(p0,v0);
 std::cout<<std::setprecision(15)<<"init="<<init<<" vsig="<<vsig<<" qr="<<qr<<' ';
 try {
 g.add(PriorFactor<Pose3>(X(0),p0,poseN));g.add(PriorFactor<Vector3>(V(0),v0,velN));g.add(PriorFactor<imuBias::ConstantBias>(B(0),b0,biasN));v.insert(X(0),p0);v.insert(V(0),v0);v.insert(B(0),b0);opt.update(g,v);g.resize(0);v.clear();std::cout<<"initial_ok ";
 for(int i=0;i<count;i++){double ax,ay,az,gx,gy,gz,dt;f>>ax>>ay>>az>>gx>>gy>>gz>>dt;pim.integrateMeasurement(Vector3(ax,ay,az),Vector3(gx,gy,gz),dt);}
 std::cout<<"integrated="<<count<<" dt="<<pim.deltaTij()<<' ';
 g.add(ImuFactor(X(0),V(0),X(1),V(1),B(0),pim));g.add(BetweenFactor<imuBias::ConstantBias>(B(0),B(1),b0,noiseModel::Diagonal::Sigmas(sqrt(pim.deltaTij())*bn)));g.add(PriorFactor<Pose3>(X(1),p1,corrN));auto predicted=pim.predict(state,b0);v.insert(X(1),predicted.pose());v.insert(V(1),predicted.v());v.insert(B(1),b0);
 opt.update(g,v);std::cout<<"update1_ok ";opt.update();std::cout<<"update2_ok vel="<<opt.calculateEstimate<Vector3>(V(1)).transpose()<<'\n';
 }catch(const std::exception&e){std::cout<<"FAILED "<<e.what()<<'\n';return 2;}
}
