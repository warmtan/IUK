#ifndef ROBOT_MODEL
#define ROBOT_MODEL

#include <rbdl/rbdl.h>
#include "RobotDefinition.hpp"

class DynModel;
class KinModel;

class RobotModel{
public:
    RobotModel();
     ~RobotModel(void);

     bool getMassInertia(MatrixXd & A) const ; //(获取质量惯性)
     bool getInverseMassInertia(MatrixXd & Ainv) const;  //(获取逆质量惯性)
     bool getGravity(VectorXd & grav) const;  //(获取重力)
     bool getCoriolis(VectorXd & coriolis) const; //(获取科里奥利力)
    //in world frame
     void getCentroidJacobian(MatrixXd & Jcent) const;  //获取质心雅可比）
     void getCentroidInertia(MatrixXd & Icent) const;    //获取获取质心惯性）
     void getCentroidMatrix(MatrixXd & Mcent) const;   //（获取质心矩阵）
    // according to "Improved computation of the humanoid centroidal dynamics and application for 
    // 根据“改进的人形质心动力学计算和应用
    // whole body control" by Patrick Wensing
    // 全身控制“帕特里克·温辛
    //when rpy not equal 0, result is a little fault!!!
    // 当 RPY 不等于 0 时，结果有点错误!!! 不明白
     void getAg_AgdotQdot(MatrixXd & Ag, MatrixXd & AgdotQdot); 

    //test dwl's code, https://github1s.com/robot-locomotion/dwl/blob/HEAD/dwl/dwl/model/WholeBodyDynamics.cpp#L204
    // 不清楚
    void getComState_dwl(const VectorXd & q, const VectorXd & qdot,
        double& total_mass, rbdl_Vec3& com_pos, rbdl_Vec3& com_vel,
        rbdl_Vec3& ang_momentum);
    
    //when rpy not equal 0, result is a little fault!!!  不清楚
    // 不清楚
    void getCentroidInertia_dwl(const VectorXd & q, MatrixXd & Icent);


     void getCoMPosition(Vec3 & com_pos) const;  //更新坐标！吗
     void getCoMVelocity(Vec3 & com_vel) const;  // 不清楚
     void getPos(int link_id, Vec3 & pos) const;  //获取link 坐标  
     void getOri(int link_id, Vec3 & rpy) const;  // 获取link 的 rpy
     void getLinearVel(int link_id, Vec3 & lin_vel) const; //获取线性速度
     void getAngularVel(int link_id, Vec3 & ang_vel) const; //获取角速度
     //void getCentroidVelocity(VectorXd & centroid_vel) const;
     void getCoMJacobian(MatrixXd & J) const;  //获取不清楚
     void getFullJacobian(int link_id, MatrixXd & J) const; //更具id 获取矩阵
     void getFullJDotQdot(int link_id, VectorXd & JDotQdot) const; //更具id 获取向量
    //matrix from world frame to body frame
     void getWorld2BodyMatrix(Mat3 & _World2Body); //获取世界to身体的旋转矩阵

     void UpdateSystem(const VectorXd & q, const VectorXd & qdot);  //更新系统 主要功能
    //according to this function, to know the number of each joint
     void PrintLinkList(); //获取link数量 从而获取关节数量
     unsigned int FindLinkId(const char* link_name); //获取link的id
protected:
    DynModel* dyn_model_;
    KinModel* kin_model_;

    RigidBodyDynamics::Model* model_;
};

#endif
