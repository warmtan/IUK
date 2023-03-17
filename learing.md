# .hpp 文件

    本质就是将.cpp的实现代码混入.h头文件当中，定义与实现都包含在同一文件，
    则该类的调用者只需要include该.hpp文件即可，无需再将cpp加入到project中进行编译。
    而实现代码将直接编译到调用者的obj文件中，不再生成单独的obj，
    采用hpp将大幅度减少调用project中的cpp文件数与编译次数，也不用再发布lib与dll文件，
    因此非常适合用来编写公用的开源库。

# hpp与cpp

    不是很严格的讲，.h文件做的是类的声明，包括类成员的定义和函数的声明，
    而.cpp文件做的类成员函数的具体实现（定义）。

# 从main.cpp出发

<!-- 定义头文件 头文件中包含程序中必需的或有用的信息 -->
    #include "model/RobotModel.hpp"
    #include "common_utils/common.hpp"

<!-- 定义一个结构体类型 modelStateData -->
<!-- 为了访问结构的成员，我们使用成员访问运算符（.） -->
    struct modelStateData{
	VectorXd _q;
	VectorXd _qdot;
    };

<!--  int main() 是主函数，程序从这里开始执行 -->
    调用class RobotModel 声明 _model = new RobotModel();

<!-- 了解一下RobotModel 类 -->
    在类中分为两类的访问修饰符public protected
    在公有成员中：构造函数的声明RobotModel()
                析构函数的声明 ~RobotModel(void);
<!-- 类的构造函数是类的一种特殊的成员函数，它会在每次创建类的新对象时执行。
     构造函数的名称与类的名称是完全相同的，
     并且不会返回任何类型，也不会返回 void。
     构造函数可用于为某些成员变量设置初始值。 -->

<!-- 类的析构函数是类的一种特殊的成员函数，它会在每次删除所创建的对象时执行。
     析构函数的名称与类的名称是完全相同的，只是在前面加了个波浪号（~）作为前缀，它不会返回任何值，也不能带有任何参数。
     析构函数有助于在跳出程序（比如关闭文件、释放内存等）前释放资源。-->

     在公有成员中定义了 bool getMassInertia(MatrixXd & A) const ; (获取质量惯性)
                     bool getInverseMassInertia(MatrixXd & Ainv) const;  (获取逆质量惯性)
                     bool getGravity(VectorXd & grav) const; (获取重力)
                     bool getCoriolis(VectorXd & coriolis) const; (获取科里奥利力)
            in world frame(在世界的框架中)
                     void getCentroidJacobian(MatrixXd & Jcent) const; （获取质心雅可比）
                     void getCentroidInertia(MatrixXd & Icent) const;  （获取获取质心惯性）
                     void getCentroidMatrix(MatrixXd & Mcent) const;   （获取质心矩阵）
                     //when rpy not equal 0, result is a little fault!!!
                     void getAg_AgdotQdot(MatrixXd & Ag, MatrixXd & AgdotQdot);

                     void getComState_dwl(const VectorXd & q, const VectorXd & qdot,
                            double& total_mass, rbdl_Vec3& com_pos, rbdl_Vec3& com_vel,
                            rbdl_Vec3& ang_momentum);

                    //when rpy not equal 0, result is a little fault!!!
                     void getCentroidInertia_dwl(const VectorXd & q, MatrixXd & Icent);

                     void getCoMPosition(Vec3 & com_pos) const;
                     void getCoMVelocity(Vec3 & com_vel) const;
                     void getPos(int link_id, Vec3 & pos) const;
                     void getOri(int link_id, Vec3 & rpy) const;
                     void getLinearVel(int link_id, Vec3 & lin_vel) const;
                     void getAngularVel(int link_id, Vec3 & ang_vel) const;
                    //void getCentroidVelocity(VectorXd & centroid_vel) const;
                     void getCoMJacobian(MatrixXd & J) const;
                    void getFullJacobian(int link_id, MatrixXd & J) const;
                    void getFullJDotQdot(int link_id, VectorXd & JDotQdot) const;
                    //matrix from world frame to body frame
                    void getWorld2BodyMatrix(Mat3 & _World2Body);

                    void UpdateSystem(const VectorXd & q, const VectorXd & qdot);
                    //according to this function, to know the number of each joint
                    void PrintLinkList();
                    unsigned int FindLinkId(const char* link_name);

# Eigen库学习

    在Eigen，所有的矩阵和向量都是Matrix模板类的对象  [2,2] 行列
    VectorXd的 [2] 列