#include "model/RobotModel.hpp"
#include "common_utils/common.hpp"

struct modelStateData{
	VectorXd _q;
	VectorXd _qdot;
};

int main(void) {
	RobotModel* _model = new RobotModel();
	//_model->PrintLinkList();
	struct modelStateData _state = {
		VectorXd::Zero(robot::num_q), 
		VectorXd::Zero(robot::num_qdot)
	};
	double mass;
	Vec3 RPY, linkPos, linkOri, linkVel, linkAngVel;
	MatrixXd cJ, cI, cI_dwl, cenInerJac, Ag_kim, Ag_wensing, AgdotQdot_wensing, H;
	VectorXd L, comvel_2, L_wensing, qdot, G;
	Vec3 compos, comvel, linmomen_dwl;
	rbdl_Vec3 angmomen_dwl, compos_dwl, comvel_dwl;
	Mat3 World2BodyInRbdl, Body2WorldMyself;
	RPY << 0.8, 0.5, 0.1;
	Mat3 R = RotZ_Matrix(RPY[2]) * RotY_Matrix(RPY[1]) * RotX_Matrix(RPY[0]);
	MatrixXd R_6D = MatrixXd::Zero(6,6);
	R_6D.block(0,0,3,3) = R;
	R_6D.block(3,3,3,3) = R;
	eigenQuaternion qua(R);
	_state._q << 0, 0, 0, qua.x(), qua.y(), qua.z(),
		0., 0., 0,
		0, 0.7, 0,
		0, 0.8, 0,
		0., 0, 0,
		qua.w();
	//xyz rpy
	_state._qdot << 0.7, 0.5, 0.2, 500, 500, 500,
		0.9, 0, 0.,
		0.9, 0, 0,
		1.8, 0, 0.7,
		8, 0, 0;
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
	
	qdot.resize(18);
	//vel: rpy xyz joint
	qdot << _state._qdot.segment(3,3), _state._qdot.segment(0,3), _state._qdot.tail(12);
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
		cout << "kim\n" << Ag_kim.block(0,0,6,12) << endl;
		cout << "wensing\n" << Ag_wensing.block(0,0,6,12) << endl;
	  
	cout << "gravity\n";
		cout << G.transpose() << endl;

	return 0;
}