
// A simple example

// Here is a simple example how one can create a meaningless model and compute the forward dynamics for it:
/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2016 Martin Felis <martin@fysx.org>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */
 
#include <iostream>
#include <vector>
 
#include <rbdl/rbdl.h>
 
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
 
int main (int argc, char* argv[]) {
    rbdl_check_api_version (RBDL_API_VERSION);
    
    Model* model = NULL;

    unsigned int body_a_id, body_b_id, body_c_id;
    Body body_a, body_b, body_c;
    Joint joint_a, joint_b, joint_c;

    model = new Model();

    model->gravity = Vector3d (0., -9.81, 0.);
 
//   // 初始化模型
//     Model* model = NULL;
//     model = new Model();
    // model->gravity = Vector3d(0., 0., -9.81);

    // 建立body和joint
    unsigned int body_1_id, body_2_id, body_3_id, body_4_id, body_5_id, body_6_id;
    Body body_1, body_2, body_3, body_4, body_5, body_6;
    Joint joint_1, joint_2, joint_3, joint_4, joint_5, joint_6;
    SpatialTransform transform1, transform2, transform3, transform4, transform5, transform6;

    // joint 1
    body_1 = Body(1., Vector3d(0.0, 0.0, 0.0), Vector3d(1., 1., 1.)); // 给定连杆的属性：质量参数、质心坐标、相对于质心坐标系下的惯性矩阵
    joint_1 = Joint(JointTypeRevoluteZ); // z轴旋转——给定关节的信息：关节类型、运动轴
    body_1_id = model->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_1, body_1); // 父类id（base=0），关节之间的坐标转换矩阵6*6，关节模型、连杆属性

    // joint 2
    body_2 = Body(1., Vector3d(105.0, 0.0, 0.0), Vector3d(1., 1., 1.));
    joint_2 = Joint(JointTypeRevoluteZ);
    body_2_id = model->AddBody(body_1_id, Xrotx(-M_PI/2.0), joint_2, body_2);

    // joint 3
    body_3 = Body(1., Vector3d(105.0, 0.0, 0.0), Vector3d(1.0, 1.0, 1.0));
    joint_3 = Joint(JointTypeRevoluteZ);
    body_3_id = model->AddBody(body_2_id, Xtrans(Vector3d(500.0, 0., 475.0)), joint_3, body_3);

    // joint 4
    body_4 = Body(1., Vector3d(105.0, 0.0, 0.0), Vector3d(1.0, 1.0, 1.0));
    joint_4 = Joint(JointTypeRevoluteZ);
    transform4 = Xrotx(-M_PI / 2.0); transform4.r = Vector3d(50.0, 370.0, 0.0);
    body_4_id = model->AddBody(body_3_id, transform4, joint_4, body_4);

    // joint 5
    body_5 = Body(1., Vector3d(105.0, 0.0, 0.0), Vector3d(1.0, 1.0, 1.0));
    joint_5 = Joint(JointTypeRevoluteZ);
    transform5 = Xrotx(M_PI / 2.0);
    body_5_id = model->AddBody(body_4_id, transform5, joint_5, body_5);

    // joint 6
    body_6 = Body(1., Vector3d(105.0, 0.0, 0.0), Vector3d(1.0, 1.0, 1.0));
    joint_6 = Joint(JointTypeRevoluteZ);
    transform6 = Xrotx(-M_PI / 2.0);
    body_6_id = model->AddBody(body_5_id, transform6, joint_6, body_6);
  
 
    VectorNd Q = VectorNd::Zero (model->q_size);
    VectorNd QDot = VectorNd::Zero (model->qdot_size);
    VectorNd Tau = VectorNd::Zero (model->qdot_size);
    VectorNd QDDot = VectorNd::Zero (model->qdot_size);

    ForwardDynamics (*model, Q, QDot, Tau, QDDot);
    std::vector <unsigned int> body_id = {body_6_id};
    VectorNd qres;
    std::vector<Vector3d> body_point = {{0.5,0.13,0.13}};	
    std::vector<Vector3d> target_pos = {{0,0,0}};
    InverseKinematics(*model, Q, body_id, body_point,target_pos,qres);
    std::cout << "关节坐标" << qres << std::endl;
    std::cout << "Q " << Q << std::endl;
    std::cout << "Q1 " << Q[1] << std::endl;
    std::cout << "Q2 " << Q[2] << std::endl;
    std::cout << "Q:" << Q.size()<<std::endl;
    std::cout << "QD:" << QDot.size() <<std::endl;
    std::cout << QDDot.transpose() << std::endl;
    delete model;

  return 0;
}