#include "DynModel.hpp"

using namespace RigidBodyDynamics::Math;

//构造函数
DynModel::DynModel(RigidBodyDynamics::Model* model){
    model_ = model;
}
//析构函数
DynModel::~DynModel(){
}
//成员函数 获取质量惯性
bool DynModel::getMassInertia(MatrixXd & a){
    a = A_;
    return true;
}
// 获取逆质量惯性
bool DynModel::getInverseMassInertia(MatrixXd & ainv){
    ainv = Ainv_;
    return true;
}
//成员函数 获取重力
bool DynModel::getGravity(VectorXd &  grav){
    grav = grav_;
    return true;
}
// 成员函数 获取科里奥利
bool DynModel::getCoriolis(VectorXd & coriolis){
    coriolis = coriolis_;
    return true;
}
// 成员函数 更新动态
// VectorXd  向量是矩阵的特殊情况，也是用矩阵定义的 向量容器
void DynModel::UpdateDynamics(const VectorXd & q, 
        const VectorXd & qdot){
    // Mass Matrix 质量矩阵
    // MatrixXd::Zero 零矩阵
    A_ = MatrixXd::Zero(model_->qdot_size, model_->qdot_size);
    // 复合刚体算法
    CompositeRigidBodyAlgorithm(*model_, q, A_, false);

    // Ainv_ = A_.inverse(); 求逆矩阵inverse
    // 伪逆矩阵
    pseudoInverse(A_, 1.e-10, Ainv_, 0);

    // 定义0向量容器
    VectorXd ZeroQdot = VectorXd::Zero(model_->qdot_size);
    // Gravity 重力 定义0向量容器
    VectorXd grav_tmp = VectorXd::Zero(model_->qdot_size);
    // 逆动力学
    InverseDynamics(*model_, q, ZeroQdot, ZeroQdot, grav_tmp);
    grav_ = grav_tmp;
    // Coriolis 科里奥利力
    VectorXd coriolis_tmp = VectorXd::Zero(model_->qdot_size);
    InverseDynamics(*model_, q, qdot, ZeroQdot, coriolis_tmp);
    // 这个获取的科里奥利力
    coriolis_ = coriolis_tmp - grav_;
}
