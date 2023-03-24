#include "model/RobotModel.hpp"
#include "common_utils/common.hpp"

struct modelStateData{
	VectorXd _q;
	VectorXd _qdot;
};

int main(void) {
	RobotModel* _model = new RobotModel();
	// 打印出来link列表
	_model->PrintLinkList();

	// Vec3 pos;
	// _model->getCoMPosition(pos);
	// cout << "pos " << pos << endl;
	struct modelStateData _state = {
		VectorXd::Zero(robot::num_q), 
		VectorXd::Zero(robot::num_qdot)
	};
	// 定义了一个double
	double mass;
	// 定义3行一列矩阵
	Vec3 RPY, linkPos, linkOri, linkVel, linkAngVel;
	// 定义了一个矩阵类型
	MatrixXd cJ, cI, cI_dwl, cenInerJac, Ag_kim, Ag_wensing, AgdotQdot_wensing, H;
	// 定义了一个向量矩阵
	VectorXd L, comvel_2, L_wensing, qdot, G;
	// 定义了3行一列矩阵
	Vec3 compos, comvel, linmomen_dwl;
	// rbdi 的3*3矩阵
	rbdl_Vec3 angmomen_dwl, compos_dwl, comvel_dwl;
	// 定义了一个3*3矩阵
	Mat3 World2BodyInRbdl, Body2WorldMyself;
	// 
	RPY << 0, 0, 0;
	// cout << RPY <<endl;
	Mat3 R = RotZ_Matrix(RPY[2]) * RotY_Matrix(RPY[1]) * RotX_Matrix(RPY[0]);
	// Mat3 R = RotX_Matrix(RPY[0]) * RotY_Matrix(RPY[1]) * RotZ_Matrix(RPY[2]);
	// cout << R << endl;
	MatrixXd R_6D = MatrixXd::Zero(6,6);
	// cout << R_6D << endl;
	R_6D.block(0,0,3,3) = R;
	// cout << R_6D <<endl;

	R_6D.block(3,3,3,3) = R;
	// // cout << R_6D <<	endl;
	eigenQuaternion qua(R);
	// _state._q << 0,0,0, qua.x(), qua.y(), qua.z(),0,-0.40,0,0.90,0.0,0.40,0.0, qua.w();
	// // 这是设置base_size坐标系
	_state._q << 0, 0.0, 0, 0.0, 0.0, 0.0, 0.0;
	// // 自认为是设置flange坐标系不是这个
	// // 这是设置广义速度变量
	// // 这一步需要求解
	_state._qdot << 2.0944, 2.0944, 2.4435,2.4435, 4.8869, 4.8869,4.8869;
	// cout << "updateSystem" << endl;
	// _model -> UpdateSystem(_state._q, _state._qdot);
	// _model->getCoMPosition(pos);
	// Vec3 a;
	// _model -> getPos(0,a);
	// cout << "pos" << pos << endl;
	// cout << "a Value :"<< a <<endl;
	// Vec3 a1;
	// _model->getPos(7,a1);
	// cout << a1 << endl;
	// _model -> UpdateSystem(_state._q, _state._qdot);
	// for (unsigned int i = 0; i < 7; i++) {
	// 		Vec3 a1;
	// 		_model->getPos(7,a1);
	// 		body_point_num << a1[0][0],a1[0][0],a1[0][0];
	// }
	vector< unsigned int > body_id_num;
	body_id_num = {7};
	// Vec3 a0,a1,a2,a3,a4,a5,a6,a7;
	// _model->getPos(0,a0);
	// _model->getPos(1,a1);
	// _model->getPos(2,a2);
	// _model->getPos(3,a3);
	// _model->getPos(4,a4);
	// _model->getPos(5,a5);
	// _model->getPos(6,a6);
	// _model->getPos(7,a7);
	// cout << "a0 value" << a0 <<endl;
	// cout << "a1 value" << a1 <<endl;
	// cout << "a2 value" << a2 <<endl;
	// cout << "a3 value" << a3 <<endl;
	// cout << "a4 value" << a4 <<endl;
	// cout << "a5 value" << a5 <<endl;
	// cout << "a6 value" << a6 <<endl;
	// cout << "a7 value" << a7 <<endl;
	vector<rbdl_Vec3>body_point_num,target_pos_num;
	body_point_num = {{0.68659854, -0.11323812,0.28814268}};
	target_pos_num = {{0,0,0}};
	// target_pos_num = {a0,a1,a2,a3,a4,a5,a6,a7};
    // vector<rbdl_Vec3>body_point_num,target_pos_num;
	// for (unsigned int i = 0; i < 8; i++) {
	// 		_model->getPos(7,a1);
	// 		body_point_num[i] << a1[0],a1[1],a1[2];
	// }

	// cout << body_point_num[1] << endl;
	// target_pos_num << 0.68659854, -0.11323812,0.28814268, 1000, 0, 1000;
	// // // body_point_num 就是 
    VectorXd qres;
	// // qinit body_id body_point target_pos qres
	// // 状态的初始猜测                     const Math::VectorNd qinit
	// // 运动学目标位置的所有物体的矢量   	 const std::vector< unsigned int >  body_id_num
	// // 要与目标位置匹配的身体局部坐标的点向量 const std::vector< Math::Vector3d > body_point_num
	// // 目标位置向量                       const std::vector< Math::Vector3d > target_pos_num
	// // 计算的逆运动学输出                  Math::VectorNd &   qres
	// cout << "1"<< endl;
    _model->RbdlIK(_state._q,body_id_num, body_point_num,target_pos_num,qres);
	// cout << qres << endl;
	// cout << "2"<< endl;
	// MatrixXd jaco1,jaco2,jaco3,jaco4,jaco5,jaco6,jaco7;
	// _model->getFullJacobian(1,jaco1);
	// cout << "关节1雅可比矩阵" << jaco1 << endl;
	// VectorXd qdot1, qdot2, qdot3, qdot4, qdot5, qdot6, qdot7;
	// _model->getFullJDotQdot(1,qdot1);
	// cout << "关节1速度" << qdot1 << endl;
	return 0;
}