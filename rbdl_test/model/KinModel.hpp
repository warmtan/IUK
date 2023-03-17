#ifndef KIN_MODEL
#define KIN_MODEL

#include "RobotDefinition.hpp"
#include <rbdl/rbdl.h>
#include <iostream>

class KinModel{
    public:
        KinModel(RigidBodyDynamics::Model* model);
        ~KinModel(void);
        //in world frame
        void getPos(int link_id, Vec3 & pos); //得到xyz
        void getOri(int link_id, Vec3 & rpy); //rpy
        void getLinearVel(int link_id, Vec3 & vel); //线速度
        void getAngularVel(int link_id, Vec3 & ang_vel); //角速度
        void getJacobian(int link_id, MatrixXd & J);
        void getJDotQdot(int link_id, VectorXd & JDotQdot);
        void getCoMJacobian  (MatrixXd & J) const;
        void getCoMPos  (Vec3 & com_pos) const;
        void getCoMVel (Vec3 & com_vel) const;
        void getCentroidInertia(MatrixXd & Icent){ Icent = Ig_; } //获取获取质心惯性）
        void getCentroidJacobian(MatrixXd & Jcent){ Jcent = Jg_; } //获取质心雅可比）
        void getCentroidMatrix(MatrixXd & Mcent){ Mcent = Ag; }  // 获取质心矩阵）
        //void getCentroidVelocity(VectorXd & centroid_vel);  
        //matrix from world frame to body frame
        //获取世界to身体的旋转矩阵
        void getWorldToBodyMatrix(Mat3 & BodyMatrix); 

        // 更运动学 主要功能
        void UpdateKinematics(const VectorXd & q, const VectorXd & qdot);

        unsigned int _find_body_idx(int id) const;
        unsigned int find_body_id(const char* link_name) const;
        void displaylinks();

        Vec3 com_pos_;
        VectorXd centroid_vel_;
    protected:
        double gravity_;
        void _UpdateCentroidFrame(const VectorXd & q, const VectorXd & qdot); //更新质心框架
        MatrixXd Ig_, Ig_2;
        MatrixXd Jg_;
        MatrixXd Ag, Ag_2;

        RigidBodyDynamics::Model* model_;
};

#endif
