#ifndef MERCURY_DYN_MODEL
#define MERCURY_DYN_MODEL

#include "common_utils/common.hpp"
#include <rbdl/rbdl.h>

class DynModel{
public:

    DynModel(RigidBodyDynamics::Model* model);
    ~DynModel(void);

    bool getMassInertia(MatrixXd & a); //获取质量惯性
    bool getInverseMassInertia(MatrixXd & ainv);
    bool getGravity(VectorXd &  grav);
    bool getCoriolis(VectorXd & coriolis);

    void UpdateDynamics(const VectorXd & q, const VectorXd & qdot);

protected:
    MatrixXd A_;
    MatrixXd Ainv_;
    VectorXd grav_;
    VectorXd coriolis_;

    RigidBodyDynamics::Model* model_;
};

#endif
