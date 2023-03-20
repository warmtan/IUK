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
	// cout << R << endl;
	// cout << qua.x() << endl;
	// cout << qua.y() << endl;
	// cout << qua.z() << endl;
	// cout << qua.w() << endl;
	// _state._q << 0.68659854,-0.11323812,0.28814268, qua.x(), qua.y(), qua.z(),0,-0.40,0,0.90,0.0,0.40,0.0, qua.w();
	_state._q << 0.0,0.0,0.0, qua.x(), qua.y(), qua.z(),0,0.0,0,0.0,0.0,0.0,0.0, qua.w();
	_state._qdot << 0.68659854, -0.11323812,0.28814268, 1000, 0, 1000,2.0944, 2.0944, 2.4435,2.4435, 4.8869, 4.8869,4.8869;
	_model -> UpdateSystem(_state._q, _state._qdot);
	_model -> getCoMPosition(compos);
	_model -> getCoMVelocity(comvel);
	_model -> getAg_AgdotQdot(Ag_wensing, AgdotQdot_wensing);
	_model -> getCentroidJacobian(cJ);
	_model -> getCentroidInertia(cI);
	_model -> getCentroidInertia_dwl(_state._q, cI_dwl);
	_model -> getCentroidMatrix(Ag_kim);
	_model -> getMassInertia(H);
	_model -> getGravity(G);

	_model -> getComState_dwl(_state._q, _state._qdot, mass, compos_dwl, comvel_dwl, angmomen_dwl);
	
	qdot.resize(13);
	cout << "qdot :" << qdot.size() << endl;
	qdot << _state._qdot.segment(3,3), _state._qdot.segment(0,3), _state._qdot.tail(7);
	L = Ag_kim * qdot;
	comvel_2 = cJ * qdot;
	linmomen_dwl = mass * comvel_dwl;
	L_wensing = Ag_wensing *  qdot;

	cout << "compos\n";
		cout << "kim\n" << compos.transpose() << endl;
		cout << "dwl\n" << compos_dwl.transpose() << endl;
	cout << "com linear vel\n";
		cout << "kim\n" << comvel.transpose() << endl;
		cout << "dwl\n" << comvel_dwl.transpose() << endl;
	cout << "com vel(Jcm * qdot)\n" << comvel_2.transpose() << endl;
	cout << "com ang momen\n";
		cout << "kim\n" << L.head(3).transpose() << endl;
		cout << "dwl\n" << angmomen_dwl.transpose() << endl;
		cout << "wensing\n" << L_wensing.head(3).transpose() << endl;
	cout << "com lin momen\n";
		cout << "kim\n" << L.tail(3) .transpose() << endl;
		cout << "dwl\n" << linmomen_dwl.transpose() << endl;
		cout << "wensing\n" << L_wensing.tail(3).transpose() << endl;
	cout << "centroidal inertia\n";
		cout << "kim\n" << cI<< endl;
		cout << "dwl\n" << cI_dwl << endl;
	cout << "Ag\n";
		cout << "kim\n" << Ag_kim.block(0,0,6,7) << endl;
		cout << "wensing\n" << Ag_wensing.block(0,0,6,7) << endl;
	  
	cout << "gravity\n";
		cout << G.transpose() << endl;

	// Vec3 a;
	// _model ->getPos(0,a);

	return 0;
}