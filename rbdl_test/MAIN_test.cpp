#include <rbdl/urdfreader.h>
#include <rbdl/rbdl.h>
#include "common_utils/common.hpp"

#define THIS_COM "/home/robotflow/IUK/rbdl_test/urdf/"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

class Solution{
public:
	Solution();
protected:
    Model* model_;
};

Solution::Solution(){
  model_ = new Model();
  rbdl_check_api_version (RBDL_API_VERSION);

  if (!Addons::URDFReadFromFile (
              THIS_COM"flexiv.urdf", model_, false, false)) {
    std::cerr << "Error loading model flexiv.urdf" << std::endl;
    abort();
  }
  //  std::cerr << model_ ->PrintLinkList()<< std::endl;
//   S1 = new Solution2(model_);
//   S1 = new Solution1(model_);

  printf("[Aliengo Model] Contructed\n");

}
int main(){
	Solution* _model = new Solution();
	std::cout << _model << std::endl;
}