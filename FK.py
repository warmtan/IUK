import numpy as np

#DH参数
DH0_armc = np.array([[0, -pi/2, 0, 0.248],
					[0,  pi/2, 0, 0    ],
					[0, -pi/2, 0, 0.305],
					[0,  pi/2, 0, 0    ],
					[0, -pi/2, 0, 0.306],
					[0,  pi/2, 0, 0    ],
					[0,  0,    0, 0.213]])
#关节极限
q_min_armc = np.array([-180, -95, -180, -95, -180, -95, -180])*pi/180
q_max_armc = np.array([180, 95, 180, 95, 180, 95, 180])*pi/180

#单关节传递矩阵
def  trans(theta,alpha,a,d):
	'''
		本函数用于求取n自由度机械臂正运动学
		输入参数为DH参数，角度单位为rad，长度单位为mm
		参数分别为theta,alpha,a,d，为0维常数值
		返回齐次传递函数矩阵
	'''
	T = np.array([[math.cos(theta), -math.sin(theta)*math.cos(alpha), math.sin(theta)*math.sin(alpha),  a*math.cos(theta)],
		[math.sin(theta), math.cos(theta)*math.cos(alpha),  -math.cos(theta)*math.sin(alpha), a*math.sin(theta)],
		[0,               math.sin(alpha),                  math.cos(alpha),                  d                ],
		[0,               0,                                0,                                  1              ]])
	return T

#返回齐次矩阵的正运动学
def fkine(theta,alpha,a,d):
	'''
		本函数用于求取n自由度机械臂正运动学
		输入参数为DH参数，角度单位为rad，长度单位为mm
		参数分别为theta,alpha,a,d，为1维常数值
		返回齐次传递函数矩阵 
	'''
	#关节自由度
	n = len(theta)
	#建立4×4的齐次传递矩阵,定义为numpy类型
	An = np.eye(4)
	for i  in range(n):
		T =  trans(theta[i],alpha[i],a[i],d[i])
		An = np.dot(An,T)  #末端到惯性坐标系传递矩阵
	return An

#输入初始时刻DH_0和相对转角,输出六维末端位姿
def fkine_euler(DH_0,qr):
	'''
		本函数用于求取n自由度机械臂正运动学
		输入参数为DH参数，角度单位为rad，长度单位为mm
		参数分别为theta,alpha,a,d，为1维常数值
		返回齐次传递函数矩阵
	'''
	#DH参数
	theta = DH_0[:, 0] + qr
	alpha = DH_0[:, 1]
	a = DH_0[:, 2]
	d = DH_0[:, 3]
	#关节自由度
	n = len(theta)
	xe = np.zeros(6)
	#建立4×4的齐次传递矩阵,定义为numpy类型
	An = np.eye(4)
	for i  in range(n):
		T =  trans(theta[i],alpha[i],a[i],d[i]) #需要加入该函数：单关节齐次矩阵
		An = np.dot(An,T)  #末端到惯性坐标系传递矩阵
	xe[0:3] = An[0:3,3]
	xe[3:6] = rot2euler_zyx(An[0:3,0:3]) #需要加入该函数：欧拉角转换函数

	return xe
#旋转矩阵转变为ZYX欧拉角
def rot2euler_zyx(Re):
	'''
		ZYX欧拉角速度变为姿态角速度转化矩阵
		input:旋转矩阵
		output:欧拉角[alpha,beta,gamma]
	'''
	euler_zyx = np.zeros(3)
	if(abs(abs(Re[2, 0]) - 1) < math.pow(10, -6)):
		if(Re[2,0] < 0):
			beta = pi/2
			alpha = np.arctan2(-Re[1,2],Re[1,1])
			gamma = 0
		else:
			beta = -pi/2
			alpha = -np.arctan2(-Re[1, 2], Re[1, 1])
			gamma = 0
	else:
		p_beta = math.asin(-Re[2,0])
		cb = np.cos(p_beta)
		alpha = math.atan2(Re[1,0]*cb,Re[0,0]*cb)
		gamma = math.atan2(Re[2,1]*cb,Re[2,2]*cb)
		if((math.sin(gamma)*Re[2,1]) < 0):
			beta = pi - p_beta
		else:
			beta = p_beta
	euler_zyx[0] = alpha
	euler_zyx[1] = beta
	euler_zyx[2] = gamma
	for i in range(3):
		if(euler_zyx[i]>=3.14 or euler_zyx[i]<=-3.14):
			euler_zyx[i] = 0.0
	return euler_zyx
#构造法求雅克比矩阵,时间0.3ms
def jacobian(DH_0,qr):
	'''
		本函数用于求取机械臂的雅克比矩阵
		input:DH_0参数，长度单位mm,角度单位red
			  qr,相对初始位置的转角
		output:J,该位置点的雅克比矩阵
	'''
	n = len(qr)
	theta = DH_0[:,0] + qr
	alpha = DH_0[:,1]
	a = DH_0[:,2]
	d = DH_0[:,3]
	
	#求取末端位置
	An = fkine(theta,alpha,a,d)  #正运动学函数
	p_n = An[0:3,3]
	J = np.zeros([6,n])
	J[3:6,n-1] = An[0:3,2]
	
	#求取其余转轴方向及位置点
	Ai = np.eye(4)
	for i in range(n-1):
		z_i = Ai[0:3,2]
		p_i = Ai[0:3,3]
		p_in = p_n - p_i
		J[0:3,i] = np.cross(z_i,p_in)
		J[3:6,i] = z_i
		Ai = np.dot(Ai, trans(theta[i], alpha[i], a[i], d[i]))
	return J

#运行时间更快0.1ms
def jeco_0(DH_0, qr):
	'''
		本函数基于雅克比迭代求解n自由度机械臂逆运动学方程
		input:DH_0 = [q_init,alpha,a,d];
			 q_ready是上一时刻的位置,单位:弧度;
		     T0e为DH坐标系确定的DH{0}坐标系与DH{6}之间的关系(目标矩阵);
		     efs求解误差阀值，默认值10^(-10)
			 i_limit迭代最大次数,默认值1000			  
		output:qq为相对与DH_q0的转动角度,单位:弧度;已处理到[-pi, pi] 之间
	'''
	#建立初时刻迭代初值
	q = DH_0[:,0] + qr
	alpha = DH_0[:,1]
	a = DH_0[:,2]
	d = DH_0[:,3]
	
	#计数及标签
	n = len(q)
						  
	#计算雅克比矩阵
	U = np.eye(4)
	Jn = np.zeros([6,n])
	T = np.zeros([4,4,n])
	for i in range(n):
		i = n - i - 1
		T[:,:,i] = trans(q[i],alpha[i],a[i],d[i])#单关节传递函数
		U = np.dot(T[:,:,i],U)
		dd = np.array([-U[0,0]*U[1,3] + U[1,0]*U[0,3],
						-U[0,1]*U[1,3] + U[1,1]*U[0,3],
						-U[0,2]*U[1,3] + U[1,2]*U[0,3]])
		Jn[0:3,i] = dd
		Jn[3:6,i] = U[2,0:3] 
	
	An = fkine(q,alpha,a,d)		#正运动学函数
	R = An[0:3,0:3]
	J_R = np.zeros([6,6])
	J_R[0:3,0:3] = R
	J_R[3:6,3:6] = R
		
	J0 = np.dot(J_R,Jn)
	return J0


#输入初始时刻DH_0和相对转角,输出六维末端位姿
def fkine_euler(DH_0,qr):
	'''
		本函数用于求取n自由度机械臂正运动学
		输入参数为DH参数，角度单位为rad，长度单位为mm
		参数分别为theta,alpha,a,d，为1维常数值
		返回齐次传递函数矩阵
	'''
	#DH参数
	theta = DH_0[:, 0] + qr
	alpha = DH_0[:, 1]
	a = DH_0[:, 2]
	d = DH_0[:, 3]
	#关节自由度
	n = len(theta)
	xe = np.zeros(6)
	#建立4×4的齐次传递矩阵,定义为numpy类型
	An = np.eye(4)
	for i  in range(n):
		T =  trans(theta[i],alpha[i],a[i],d[i]) #需要加入该函数：单关节齐次矩阵
		An = np.dot(An,T)  #末端到惯性坐标系传递矩阵
	xe[0:3] = An[0:3,3]
	xe[3:6] = rot2euler_zyx(An[0:3,0:3]) #需要加入该函数：欧拉角转换函数

	return xe


#***基于雅克比矩阵迭代求解逆运动学***#
def iterate_ikine(DH_0, q_ready, T0e, efs = pow(10,-12), i_max = 1000):
	'''
		本函数基于雅克比迭代求解n自由度机械臂逆运动学方程
		input:DH_0 = [q_init,alpha,a,d];
			 q_ready是上一时刻的位置,单位:弧度;
		     T0e为DH坐标系确定的DH{0}坐标系与DH{6}之间的关系(目标矩阵);
		     efs求解误差阀值，默认值10^(-10)
			 i_limit迭代最大次数,默认值1000			  
		output:qq为相对与DH_q0的转动角度,单位:弧度;已处理到[-pi, pi] 之间
	'''
	#建立初时刻迭代初值
	q_r = DH_0[:,0] + q_ready
	alpha = DH_0[:,1]
	a = DH_0[:,2]
	d = DH_0[:,3]
	
	#计数及标签
	n = len(q_r)
	deltaQ = 1  
	temp_count = 0
	
	#迭代循环求解
	while (deltaQ > efs):
		
		#求解正运动学
		An = np.eye(4)
		T = np.zeros([4,4,n])
		
		for i in range(n):
			T[:,:,i] = trans(q_r[i],alpha[i],a[i],d[i])
			An = np.dot(An,T[:,:,i])
			
		#计算末端误差
		dA = np.zeros(6)
		dA[0:3] = T0e[0:3,3] - An[0:3,3]
		dA[3:6] = 0.5*(np.cross(An[0:3,0],T0e[0:3,0]) + np.cross(An[0:3,1],T0e[0:3,1]) 
				+ np.cross(An[0:3,2],T0e[0:3,2]))
	
		#print dA						  
		#计算雅克比矩阵
		U = np.eye(4)
		Jn = np.zeros([6,n])
		for i in range(n):
			i = n - i - 1
			U = np.dot(T[:,:,i],U)
			
			dd = np.array([ -U[0,0]*U[1,3] + U[1,0]*U[0,3],
							-U[0,1]*U[1,3] + U[1,1]*U[0,3],
							-U[0,2]*U[1,3] + U[1,2]*U[0,3]])
			Jn[0:3,i] = dd
			Jn[3:6,i] = U[2,0:3]
			
		R = An[0:3,0:3]
		J_R = np.zeros([6,6])
		J_R[0:3,0:3] = R
		J_R[3:6,3:6] = R
		
		J0 = np.dot(J_R,Jn)
		#求取关节角关节角度偏差值
		dq = np.dot(np.linalg.pinv(J0),dA)
		q_r = q_r + dq
		deltaQ = np.linalg.norm(dq)
		temp_count =temp_count + 1
		if (temp_count > i_max):
			print("Solution wouldn't converge")
			return q_ready

	q_tmp = q_r - DH_0[:,0]		
	q = bf.qq_choose(q_tmp) #选择函数
	return q


#==========================通用运动学类======================#
class GeneralKinematic(object):
	'''
	函数依赖math和numpy
	'''
	def __init__(self, DH_0,q_min=rp.q_min, q_max=rp.q_max):
		self.DH_0 = DH_0
		self.theta = DH_0[:, 0]
		self.alpha = DH_0[:, 1]
		self.a = DH_0[:, 2]
		self.d = DH_0[:, 3]
		self.q_min = q_min
		self.q_max = q_max

		self.n = len(self.theta)

	#相邻关节传递矩阵
	def trans(self, theta, alpha, a, d):
		T = np.array([[math.cos(theta), -math.sin(theta) * math.cos(alpha),
					   math.sin(theta) * math.sin(alpha), a * math.cos(theta)],
					  [math.sin(theta), math.cos(theta) * math.cos(alpha),
					   -math.cos(theta) * math.sin(alpha), a * math.sin(theta)],
					  [0, math.sin(alpha), math.cos(alpha), d],
					  [0, 0, 0, 1]])
		return T

	# ZYX欧拉角转变为旋转矩阵
	def euler_zyx2rot(self,phi):
		'''
			ZYX欧拉角转变为旋转矩阵
			input:欧拉角
			output:旋转矩阵
		'''
		R = np.array([[np.cos(phi[0]) * np.cos(phi[1]),np.cos(phi[0]) * np.sin(phi[1]) * np.sin(phi[2]) - np.sin(phi[0]) * np.cos(phi[2]),
					   np.cos(phi[0]) * np.sin(phi[1]) * np.cos(phi[2]) + np.sin(phi[0]) * np.sin(phi[2])],
					  [np.sin(phi[0]) * np.cos(phi[1]),np.sin(phi[0]) * np.sin(phi[1]) * np.sin(phi[2]) + np.cos(phi[0]) * np.cos(phi[2]),
					   np.sin(phi[0]) * np.sin(phi[1]) * np.cos(phi[2]) - np.cos(phi[0]) * np.sin(phi[2])],
					  [-np.sin(phi[0]), np.cos(phi[1]) * np.sin(phi[2]), np.cos(phi[1]) * np.cos(phi[2])]])
		return R

	# 旋转矩阵转变为ZYX欧拉角
	def rot2euler_zyx(self, Re):
		'''
			ZYX欧拉角速度变为姿态角速度转化矩阵
			input:旋转矩阵
			output:欧拉角[alpha,beta,gamma]
		'''
		euler_zyx = np.zeros(3)
		if(abs(abs(Re[2, 0]) - 1) < math.pow(10, -6)):
			if(Re[2,0] < 0):
				beta = pi/2
				alpha = np.arctan2(-Re[1,2],Re[1,1])
				gamma = 0
			else:
				beta = -pi/2
				alpha = -np.arctan2(-Re[1, 2], Re[1, 1])
				gamma = 0
		else:
			p_beta = math.asin(-Re[2,0])
			cb = np.cos(p_beta)
			alpha = math.atan2(Re[1,0]*cb,Re[0,0]*cb)
			gamma = math.atan2(Re[2,1]*cb,Re[2,2]*cb)
			if((math.sin(gamma)*Re[2,1]) < 0):
				beta = pi - p_beta
			else:
				beta = p_beta
		euler_zyx[0] = alpha
		euler_zyx[1] = beta
		euler_zyx[2] = gamma
		for i in range(3):
			if(euler_zyx[i]>=3.14 or euler_zyx[i]<=-3.14):
				euler_zyx[i] = 0.0
		return euler_zyx

	# 将关节角计算到正负pi
	def qq_choose(self, qq):
		'''
			本函数用于选着关节角范围
			input:qq为计算出的关节角
			output:q关节角范围[-pi,pi]
		'''
		q = np.copy(qq)
		for i in range(self.n):
			while (q[i] > math.pi):
				q[i] = q[i] - 2 * math.pi
			while (q[i] < - math.pi):
				q[i] = q[i] + 2 * math.pi
		return q

	#正运动学,返回齐次矩阵
	def fkine(self, qr):
		An = np.eye(4)
		for i in range(self.n):
			T = self.trans(self.theta[i] + qr[i], self.alpha[i], self.a[i], self.d[i])
			An = np.dot(An, T)  # 末端到惯性坐标系传递矩阵
		return An

	#正运动学,输出六维末端位姿,姿态用zyx欧拉角表示
	def fkine_euler(self, qr):
		xe = np.zeros(6)
		An = np.eye(4)
		for i in range(self.n):
			T = self.trans(self.theta[i] + qr[i], self.alpha[i], self.a[i], self.d[i])
			An = np.dot(An, T)  # 末端到惯性坐标系传递矩阵
		xe[0:3] = An[0:3, 3]
		xe[3:6] = self.rot2euler_zyx(An[0:3, 0:3])
		return xe
		
	#求取雅克比矩阵
	def jeco(self, qr):
		# 计算雅克比矩阵
		U = np.eye(4)
		Jn = np.zeros([6, self.n])
		T = np.zeros([4, 4, self.n])
		for i in range(self.n):
			i = self.n - i - 1
			T[:, :, i] = self.trans(self.theta[i] + qr[i], self.alpha[i], self.a[i], self.d[i])
			U = np.dot(T[:, :, i], U)
			dd = np.array([-U[0, 0] * U[1, 3] + U[1, 0] * U[0, 3],
						   -U[0, 1] * U[1, 3] + U[1, 1] * U[0, 3],
						   -U[0, 2] * U[1, 3] + U[1, 2] * U[0, 3]])
			Jn[0:3, i] = dd
			Jn[3:6, i] = U[2, 0:3]

		An = self.fkine(qr)
		R = An[0:3, 0:3]
		J_R = np.zeros([6, 6])
		J_R[0:3, 0:3] = R
		J_R[3:6, 3:6] = R

		J0 = np.dot(J_R, Jn)
		return J0

	# ***基于雅克比矩阵迭代求解逆运动学***#
	def iterate_ikine(self, q_guess, Te, efs=pow(10, -12), i_max=1000):
		'''
			本函数基于雅克比迭代求解n自由度机械臂逆运动学方程
				 q_ready是上一时刻的位置,单位:弧度;
			     T0e为DH坐标系确定的DH{0}坐标系与DH{6}之间的关系(目标矩阵);
			     efs求解误差阀值，默认值10^(-10)
				 i_limit迭代最大次数,默认值1000
			output:qq为相对与DH_q0的转动角度,单位:弧度;已处理到[-pi, pi] 之间
		'''
		# 建立初时刻迭代初值
		q_r =self.theta + q_guess

		# 计数及标签
		deltaQ = 1
		temp_count = 0

		# 迭代循环求解
		while (deltaQ > efs):

			# 求解正运动学
			An = np.eye(4)
			T = np.zeros([4, 4, self.n])

			for i in range(self.n):
				T[:, :, i] = self.trans(q_r[i], self.alpha[i], self.a[i], self.d[i])
				An = np.dot(An, T[:, :, i])

			# 计算末端误差
			dA = np.zeros(6)
			dA[0:3] = Te[0:3, 3] - An[0:3, 3]
			dA[3:6] = 0.5 * (np.cross(An[0:3, 0], Te[0:3, 0]) + np.cross(An[0:3, 1], Te[0:3, 1])
							 + np.cross(An[0:3, 2], Te[0:3, 2]))

			# 计算雅克比矩阵
			U = np.eye(4)
			Jn = np.zeros([6, self.n])
			for i in range(self.n):
				i = self.n - i - 1
				U = np.dot(T[:, :, i], U)

				dd = np.array([-U[0, 0] * U[1, 3] + U[1, 0] * U[0, 3],
							   -U[0, 1] * U[1, 3] + U[1, 1] * U[0, 3],
							   -U[0, 2] * U[1, 3] + U[1, 2] * U[0, 3]])
				Jn[0:3, i] = dd
				Jn[3:6, i] = U[2, 0:3]

			R = An[0:3, 0:3]
			J_R = np.zeros([6, 6])
			J_R[0:3, 0:3] = R
			J_R[3:6, 3:6] = R

			J0 = np.dot(J_R, Jn)
			# 求取关节角关节角度偏差值
			dq = np.dot(np.linalg.pinv(J0), dA)
			q_r = q_r + dq
			deltaQ = np.linalg.norm(dq)
			temp_count = temp_count + 1
			if (temp_count > i_max):
				print("Solution wouldn't converge")
				return q_guess

		q_tmp = q_r - self.theta
		q = self.qq_choose(q_tmp)
		return q

	#机械臂关节极限判断，返回值为0或1
	def exceed_joint_limit(self, qq, q_min, q_max):
		'''
			判断关节角是否超出限制
			input:关节角，关节角范围
			outpu：0,未超出，1超出
		'''
		n = len(qq)
		limit = False
		for i in range(n):
			if((qq[i] < q_min[i]) or (qq[i] > q_max[i])):
				print ("第", i+1, "关节超出极限:", qq[i]*180/np.pi)
				limit = True
				break
		return limit
	#带关节限制
	def iterate_ikine_limit_xyz(self, q_guess, Xe):
		Te = np.eye(4)
		Te[0:3, 0:3] = self.euler_zyx2rot(Xe[3:])
		Te[0:3, 3] = Xe[:3]
		print ("Te:", Te)
		qr = self.iterate_ikine(q_guess, Te)
		flag = self.exceed_joint_limit(qr ,self.q_min, self.q_max)
		if(flag):
			#print "flag:", flag
			qr = np.copy(q_guess)
		return qr

	# 带关节限制
	def iterate_ikine_limit(self, q_guess, Te):
		qr = self.iterate_ikine(q_guess, Te)
		flag = self.exceed_joint_limit(qr, self.q_min, self.q_max)
		if (flag):
			# print "flag:", flag
			qr = np.copy(q_guess)
		return qr
