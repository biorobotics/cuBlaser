//
// Created by dcheng on 9/8/20.
//

#include <Eigen/Dense>
#include <iostream>
#include <cmath>

using namespace std;
using namespace Eigen;
int main(int argc, char** argv)
{
  double quat[5] = {0, 0, 0, sin(M_PI / 4), cos(M_PI / 4)};
  Eigen::Quaterniond q = Eigen::Map<Eigen::Quaterniond>(quat);
  Eigen::Quaterniond q2 = Eigen::Map<Eigen::Quaterniond>(quat + 1);
  cout << "x " << q.x() << ", y " << q.y() << ", z " << q.z() << ", w " << q.w() << endl;
  cout << "x " << q2.x() << ", y " << q2.y() << ", z " << q2.z() << ", w " << q.w() << endl;

  Vector3d v = Map<Vector3d>(quat + 1);
  cout << v << endl;
}