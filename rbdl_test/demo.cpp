#include "model/RobotModel.hpp"
#include "common_utils/common.hpp"

struct modelStateData{
	VectorXd _q;
	VectorXd _qdot;
};

int main(void) {
	RobotModel* _model = new RobotModel();
	_model->PrintLinkList();
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
	// cout << R_6D <<	endl;
	eigenQuaternion qua(R);
	// _state._q << 0.68659854,-0.11323812,0.28814268, qua.x(), qua.y(), qua.z(),0,-0.40,0,0.90,0.0,0.40,0.0, qua.w();
	_state._q << 0.0,0.0,0.0, qua.x(), qua.y(), qua.z(),0,0.0,0,0.0,0.0,0.0,0.0, qua.w();
	_state._qdot << 0.68659854, -0.11323812,0.28814268, 1000, 0, 1000,2.0944, 2.0944, 2.4435,2.4435, 4.8869, 4.8869,4.8869;

	vector< unsigned int > body_id_num;
	body_id_num = {2,3,4,5,6,7,8,9,2147483647};
    vector<rbdl_Vec3>body_point_num,target_pos_num;
	// body_point_num 就是 
    VectorXd qres;
	// qinit body_id body_point target_pos qres
    // _model->RbdlIK(_state._q,body_id_num, body_point_num, target_pos_num,qres);

	
	return 0;
}