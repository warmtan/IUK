#include "KinModel.hpp"
#include <iostream>
#include <vector>
using namespace RigidBodyDynamics::Math;
using namespace RigidBodyDynamics;

//extern variables in RobotDefinition.hpp
// 在RobotDefinition.hpp定义外置变量
const char* link_name[] =
{
    "base_link",
    "link1","link2","link3",
    "link4","link5","link6",
    "link7","flange",
};

// 构造函数 初始化重力
KinModel::KinModel( RigidBodyDynamics::Model* model):
    gravity_(9.81) //重力
{
    model_ = model;
    // 定义一个6*6的0矩阵
    Ig_ = MatrixXd::Zero(6,6);
    // 定义一个6*model_->qdot_size的0矩阵
    Jg_ = MatrixXd::Zero(6, model_->qdot_size);
}
// 析构函数
KinModel::~KinModel(){
}

//成员函数 定义个更新运动学
void KinModel::UpdateKinematics(const VectorXd & q, 
        const VectorXd & qdot){
    _UpdateCentroidFrame(q, qdot);
}
// ik
bool KinModel::IK(const VectorNd &  Qinit_num,
const std::vector< unsigned int > & body_id_num, std::vector<Vector3d > & body_point_num,
const std::vector< Vector3d > & target_pos_num,VectorNd & Qres_num ){
    cout << "km1" <<endl;
    return InverseKinematics(*model_,Qinit_num,body_id_num,body_point_num, target_pos_num,Qres_num);
}



/********mainly reference to "Dynamic behaviors on the NAO robot with closed-loop 
         whole body operational space control" by Donghyun Kim**************************/
//主要参考《闭环NAO机器人动态行为》全身操作空间控制“，作者：金东贤
//  

// 成员函数更新质心框架
// 推测是一个xyz xyz 
void KinModel::_UpdateCentroidFrame(const VectorXd & q,
        const VectorXd & qdot){
    // 初始化一个double
    double mass;
    // 初始化一个3*1矩阵
    Vec3 zero_vector;
    // 填充0 
    zero_vector.setZero();

    // 初始化一个3*1矩阵
    Vec3 com_pos;
    Vec3 cm;
    Vec3 link_pos;
    Vec3 p_g;
    // 获取科波斯！！吗  质量的中心吗
    getCoMPos(com_pos);
    // 
    cout << "质心的转置输出 kim_compos:\n" << com_pos.transpose() << "\n\n";
    // 质量的中心
    com_pos_ = com_pos;

    MatrixXd Xg_inv = MatrixXd::Zero(6, 6);  //6*6矩阵
    MatrixXd link_X_com = MatrixXd::Zero(6, 6); ////6*6矩阵
    Ig_.setZero(); //6*6矩阵设为0
    cout <<"model_ ->qdot_size " << model_->qdot_size << endl;
    // 6 * 13 矩阵Ag
    Ag = MatrixXd::Zero(6, model_->qdot_size);
    // 6*6矩阵
    Ig_2 = MatrixXd::Zero(6,6);
    // 6*13 矩阵
    Ag_2 = MatrixXd::Zero(6, model_->qdot_size);
    std::cout << "model_->qdot_size: " << model_->qdot_size << "\n\n";
    // 6 * 6 矩阵
    MatrixXd I = MatrixXd::Zero(6, 6);
    // 6 * 13
    MatrixXd Jsp = MatrixXd::Zero(6, model_->qdot_size);

    int start_idx = _find_body_idx(robot_link::base_link);
    cout << "start_idx: " << start_idx << endl;
    // 3*3旋转矩阵
    Matrix3d p;
    Matrix3d cmm;
    Matrix3d R;

    cout << "model_ ->mBodies " << model_->mBodies.size() << endl;
    // model_->mBodies.size() 10
    for (int i(start_idx); i<model_->mBodies.size(); ++i){
        // 一个正交 3x3 矩阵，将向量从基本坐标旋转到身体坐标。
        R = CalcBodyWorldOrientation(*model_, q, i, false);
        // cout << i << ":" << R << endl;

        // 一个 3-D 向量，其点的坐标在基坐标中 
        link_pos = CalcBodyToBaseCoordinates ( *model_, q, i, zero_vector, false);

        Jsp.setZero(); //6 *13矩阵设置为0

        // 结果将通过 G 参数返回，并表示在主体原点表示的主体雅可比行列式。
        CalcBodySpatialJacobian( *model_, q, i, Jsp, false);

        // 质量
        mass = model_->mBodies[i].mMass;

        // 6*6 矩阵设置0 
        I.setZero();

        // 重心/质心
        cm = model_->mBodies[i].mCenterOfMass;
        cout << "m: " << mass << " mass center: " << cm.transpose() << "\n\n";
        cmm = vecCross(cm);
        cout << "cmm:" << cmm << endl;
        I.setZero();//6*6 矩阵设置0 
        // model_->mBodies[i].mInertia  质心 关心局
        I.block(0, 0, 3, 3) = model_->mBodies[i].mInertia + mass * cmm * cmm.transpose();
        I.block(0,3, 3,3) = mass * cmm;
        I.block(3,0, 3,3) = mass * cmm.transpose();
        I.block(3, 3, 3, 3) = mass * MatrixXd::Identity(3,3);

        link_X_com.block(0,0,3,3) = R;
        link_X_com.block(3,3,3,3) = R;
        link_X_com.block(3,0,3,3) = -R * vecCross(link_pos - com_pos);
        Ig_ = Ig_ + link_X_com.transpose() * I * link_X_com;
        //xyz rpy joint
        Ag = Ag + link_X_com.transpose() * I * Jsp;
        //kim's code, not intuitive
        // p_g = R * (com_pos - link_pos);
        // p = vecCross(p_g);
        // Xg_inv.block(0,0, 3,3) = R;
        // Xg_inv.block(3,3, 3,3) = R;
        // Xg_inv.block(3,0, 3,3) = p * R;
        // Ig_2 = Ig_2 + Xg_inv.transpose() * I * Xg_inv;
        // Ag_2 = Ag_2 + Xg_inv.transpose() * I * Jsp;
    }
    // bool Ig_equal, Ag_equal;
    // Ig_equal = Ig_.isApprox(Ig_2, 1e-12);
    // Ag_equal = Ag.isApprox(Ag_2, 1e-12);
    // cout << "is equal to kim\n";
    // cout << "Ig: " << Ig_equal << "  Ag: " << Ag_equal << endl;
    MatrixXd lin_mat = Ag.block(0,0,3,6);
    MatrixXd ang_mat = Ag.block(3,0,3,6);
    //moemntum: rpy xyz
    //vel: rpy xyz joint
    Ag.block(0,0,3,6) << lin_mat.rightCols(3), lin_mat.leftCols(3);
    Ag.block(3,0,3,6) << ang_mat.rightCols(3), ang_mat.leftCols(3);
    Jg_ = Ig_.inverse() * Ag;
    cout << "Jg_" << Jg_ << endl;

    //centroid_vel_ = Jg_ * qdot;
}
// 不清楚
void KinModel::getCoMJacobian(MatrixXd & Jcom) const {
    Vec3 zero_vector = Vec3::Zero();
    VectorXd q(robot::num_q);

    Jcom = MatrixXd::Zero(3, model_->qdot_size);
    MatrixNd J(3, model_->qdot_size);

    double mass;
    double tot_mass(0.0);
    int start_idx = _find_body_idx(robot_link::base_link);

    for (int i(start_idx); i< model_->mBodies.size() ; ++i){
        mass = model_->mBodies[i].mMass;
        // CoM Jacobian Update
        J.setZero();
        CalcPointJacobian(*model_, q, i, model_->mBodies[i].mCenterOfMass, J, false);
        Jcom +=  mass * J;
        tot_mass += mass;
    }
    Jcom /= tot_mass;
}
// 成员函数 输入一个3*1的矩阵  获取坐标更新 ! 更新每一个link的相对位置
// 有问题
void KinModel::getCoMPos(Vec3 & CoM_pos)const {
    // 初始化一个3*1的0矩阵
    Vec3 zero_vector = Vec3::Zero();
    // 初始化numq个0向量
    VectorXd q(robot::num_q);
    // 3*1 初始化0
    CoM_pos.setZero();
    // 3*1 初始化0
    Vec3 link_pos;

    // 定义这个id整数
    int start_idx = _find_body_idx(robot_link::base_link);
    cout << "调用成员函数getCoMPos-start_idx:" << start_idx <<endl;
    double mass;
    double tot_mass(0.0);
    cout << "这个是links 数量吗 model_->mBodies.size(): " << model_->mBodies.size() << endl;
    for (int i(start_idx); i< model_->mBodies.size() ; ++i){
        mass = model_->mBodies[i].mMass;

        cout << "mass = " << mass << endl;

        // CoM position Update
        link_pos = CalcBodyToBaseCoordinates ( *model_, q, i, 
                model_->mBodies[i].mCenterOfMass, false);
        cout <<  "link_pos "<< i << " : " <<link_pos << endl;
        CoM_pos += mass * link_pos;
        tot_mass += mass;
    }
    CoM_pos /= tot_mass;
    cout << "质量心位置CoM_pos: " << CoM_pos << endl;
}

// 不清楚
void KinModel::getCoMVel(Vec3 & CoM_vel) const {
    VectorXd q, qdot;
    int start_idx = _find_body_idx(robot_link::base_link);
    CoM_vel.setZero();
    Vec3 link_vel;

    Vec3 zero_vector = Vec3::Zero();
    double mass;
    double tot_mass(0.0);
    for (int i(start_idx); i< model_->mBodies.size() ; ++i){
        mass = model_->mBodies[i].mMass;

        // CoM velocity Update
        link_vel = CalcPointVelocity ( *model_, q, qdot, i, model_->mBodies[i].mCenterOfMass, false);
        //cout << link_vel << endl;
        CoM_vel += mass * link_vel;
        tot_mass += mass;
    }
    CoM_vel /= tot_mass;
}

// 获取id 的 xyz
void KinModel::getPos(int link_id, Vec3 & pos){
    Vec3 zero;
    Matrix3d R;
    VectorXd q(robot::num_q);

    int bodyid = _find_body_idx(link_id);
    if(bodyid >=model_->fixed_body_discriminator){
        zero = model_->mFixedBodies[bodyid - model_->fixed_body_discriminator].mCenterOfMass;
    }
    else{
        zero =  model_->mBodies[bodyid].mCenterOfMass;
    }

    pos = CalcBodyToBaseCoordinates(*model_, q, _find_body_idx(link_id), zero, false);

    // cout << pos << endl;
    // R = CalcBodyWorldOrientation( *model_, q, _find_body_idx(link_id), false);
    // pos = R * pos;

}

// 获取link 的 rpy
void KinModel::getOri(int link_id, Vec3 & rpy){
    Matrix3d R;
    eigenQuaternion ori;
    VectorXd q(robot::num_q);
    R = CalcBodyWorldOrientation( *model_, q, _find_body_idx(link_id), false);
    ori = R.transpose();

    if(ori.w() < 0.){
        ori.w() *= (-1.);
        ori.x() *= (-1.);
        ori.y() *= (-1.);
        ori.z() *= (-1.);
    }
    quaToRpy(ori, rpy[2], rpy[1], rpy[0]);
    cout << ori.w() << endl;
    cout << ori.x() << endl;
    cout << ori.y() << endl;
    cout << ori.z() << endl;
}

void KinModel::getLinearVel(int link_id, Vec3 & vel){  //线速度
    Vec3 zero;
    VectorXd q, qdot;

    int bodyid = _find_body_idx(link_id);
    if(bodyid >=model_->fixed_body_discriminator){
        zero = model_->mFixedBodies[bodyid - model_->fixed_body_discriminator].mCenterOfMass;
    }
    else{
        zero =  model_->mBodies[bodyid].mCenterOfMass;
    }

    vel = CalcPointVelocity ( *model_, q, qdot, _find_body_idx(link_id), zero, false);
    cout << "vel: " << vel << endl;
}

void KinModel::getAngularVel(int link_id, Vec3 & ang_vel){
    // 角速度
    unsigned int bodyid = _find_body_idx(link_id);
    VectorXd vel, q, qdot;

    if(bodyid >=model_->fixed_body_discriminator){
        vel = CalcPointVelocity6D(*model_, q, qdot, bodyid,
                model_->mFixedBodies[bodyid - model_->fixed_body_discriminator].mCenterOfMass, false);
    }
    else{
        vel = CalcPointVelocity6D(*model_, q, qdot, bodyid,
                model_->mBodies[bodyid].mCenterOfMass, false);
    }
    ang_vel = vel.head(3);

    cout << "Angle: " << ang_vel << endl;
}

void KinModel::getJacobian(int link_id, MatrixXd &J){
    VectorXd q(robot::num_q);
    J = MatrixXd::Zero(6, model_->qdot_size);

    unsigned int bodyid = _find_body_idx(link_id);
    Vec3 zero_vector = Vec3::Zero();
    
    if(bodyid >=model_->fixed_body_discriminator){
        CalcPointJacobian6D(*model_, q, bodyid,
                model_->mFixedBodies
                [bodyid - model_->fixed_body_discriminator].mCenterOfMass,
                J, false);
    }
    else{
        CalcPointJacobian6D(*model_, q, bodyid,
                model_->mBodies[bodyid].mCenterOfMass,
                J, false);
    }
    // 这将把雅可比框架从世界框架转变为身体框架
    /*************this will transform the jacobian from world frame to body frame********/

    // Eigen::Matrix<double, 6, 6> World2Body6D;
    // Eigen::Matrix3d World2Body;
    // World2Body6D.setZero(); World2Body.setZero();
    // World2Body = CalcBodyWorldOrientation( *model_, q, _find_body_idx(link_id), false);
    // World2Body6D.block(0,0,3,3) = World2Body;
    // World2Body6D.block(3,3,3,3) = World2Body;
    
    // J = World2Body6D * J;
}

void KinModel::getJDotQdot(int link_id, VectorXd & JDotQdot){
    VectorXd q, qdot, qddot;

    unsigned int bodyid = _find_body_idx(link_id);
    if(bodyid >=model_->fixed_body_discriminator){
        JDotQdot = CalcPointAcceleration6D(*model_, q, qdot, qddot, bodyid,
                model_->mFixedBodies
                [bodyid - model_->fixed_body_discriminator].mCenterOfMass, false);
    }
    else{
        JDotQdot = CalcPointAcceleration6D(*model_, q, qdot, qddot, bodyid,
                model_->mBodies[bodyid].mCenterOfMass, false);
    }
    JDotQdot[5] -= gravity_;
}

void KinModel::getWorldToBodyMatrix(Mat3 & BodyMatrix){
    VectorXd q;
    int body_idx = _find_body_idx(robot_link::base_link);
    cout << body_idx <<endl;

    BodyMatrix = CalcBodyWorldOrientation(*model_, q, body_idx, false);
}
// find body idx 
unsigned int KinModel::_find_body_idx(int id) const {
    switch(id){
        case robot_link::base_link:
            return model_->GetBodyId("base_link");
        case robot_link::link1:
            return model_->GetBodyId("link1");
        case robot_link::link2:
            return model_->GetBodyId("link2");
        case robot_link::link3:
            return model_->GetBodyId("link3");
        case robot_link::link4:
            return model_->GetBodyId("link4");
        case robot_link::link5:
            return model_->GetBodyId("link5");
        case robot_link::link6:
            return model_->GetBodyId("link6");
        case robot_link::link7:
            return model_->GetBodyId("link7");
        case robot_link::flange:
            return model_->GetBodyId("flange");
        default:
            std::cout << "unkonown id" << std::endl;
    }
    return 0;
}

unsigned int KinModel::find_body_id(const char* link_name) const{
    unsigned int body_id;
    body_id = model_->GetBodyId(link_name);
    cout << body_id << endl;

    return body_id;
}
//according to this function, to know the number of each joint
// 根据这个功能，要知道每个关节的数量
void KinModel::displaylinks() {
    Vec3 a;
    int linklist_len;
    linklist_len = sizeof(link_name)/sizeof(link_name[0]);
    vector<const char*> link_name_list(link_name, link_name + linklist_len);
    vector<int> body_id_list;
    int body_id;
    for (int i(0); i < link_name_list.size(); ++i) {
        
        body_id = model_->GetBodyId(link_name_list[i]);
        body_id_list.push_back(body_id);
    }
    for (int i(0); i < body_id_list.size(); ++i) {
        cout << link_name_list[i] << ": " << body_id_list[i] << endl;
    }
}