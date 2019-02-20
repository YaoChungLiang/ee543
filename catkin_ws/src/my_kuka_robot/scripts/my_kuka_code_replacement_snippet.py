    def performIK(self, pose):
        # Step 0: Extract end-effector position and orientation from request
        px = pose.position.x
        py = pose.position.y
        pz = pose.position.z
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([pose.orientation.x, pose.orientation.y,pose.orientation.z, pose.orientation.w])

	# ---------------------(1) Base to WC---------------------
	# Step 1: Find the position of wrist center Owc
        pg_0 = Matrix([[px], [py], [pz]])
        p_quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
	    d_g = s[d7] 
        R0_g = tf.transformations.quaternion_matrix(p_quat)
        R0_g = R0_g[0:3, 0:3]  
        z_g = np.dot(self.R_corr.tolist(), ([[0], [0], [1]]))

	# TODO: find wrist center position rwc_0
	#	hint1: we want rwc_0 to be a Matrix type, and we can cast data to matrix type by doing Matrix(data)
	#	hint2: the function np.dot does matrix multiplication for us.
        rwc_0 = # something

        debug_log("        Wrist Center : {}".format(rwc_0.tolist()))


        # Step 2: Calculate theta1 given Owc
	# TODO: find theta1
        #       theta1 = atan2(something, something).evalf()

        # Step 3: Define triangle (O2, O3, Owc)
        pO2_0 = Matrix([[s[a1]], [0], [s[d1]]])

        pO2_0 = rot_z(theta1)* pO2_0
        debug_log("        Link 2 position : {}".format(pO2_0.tolist()))

        pO2towc_0 = rwc_0 - pO2_0
	beta_prime = atan2(s[a3], s[d4])

        X2_prime = self.T0_2.subs({th1:theta1, th2:0}).dot([[1], [0], [0], [0]])
        z2_prime = self.T0_2.subs({th1:theta1}).dot([[0], [0], [1], [0]])

	# TODO: find the 3 edge lengths, directions and angles for this triangle
	# 	hint1: all 3 edge lengths can be found with either s[index] or information above
        #	hint2: all 3 edge directions can be interpreted as a vector of unit length
	#       hint3: all 3 angles are derived from equation: A*A+B*B-C*C = 2*A*B*cos(thetaC)
	# 	hint4: functions that might be helpful
	#	       v.normalized(): gives you a unit length vector in v direction
	#	       v.norm(): gives you the length of vector v
	#	       tf.transformations.rotation_matrix(angle,axis): gives you a 4x4 matrix that does angle axis rotation
	#	       np.arccos(float(data)): casting data to "float" ensures precision
	# 	these information will help you solve for theta2 and theta3!
        #	(please add comments to help us understand your code)

	# TODO: find pO3
        pO3 = pO2_0 + # something
        debug_log("        Link 3 position : {}".format(pO3.tolist()))


	# Step 4: Find theta2 from triangle
	# TODO: find theta2
	#       hint: the dot product of two vectors has the math form: 
	#	dot(v1,v2) = |v1|*|v2|*cos(theta)
	# 	so the angle theta between the two vectors can be depricted as: 
	#	theta = arccos(dot(v1,v2)/(|v1|*|v2|))
        theta2 = # something

        # Step 5: Find theta3 from triangle
	# TODO: find theta3 
	# 	hint: you can use (something).evalf() to ensure precision
        theta3 = # (something).evalf()


	# ---------------------(2) WC to end-effector---------------------
	# Step 1: Symbolic form of R3_6 (skip: try offline)
	# You can try doing "(T3_4*T4_5*T5_6)[:3,:3]" and see result.

	# Step 2: Numeric form of R3_6
        R0_3 = self.T0_3[0:3,0:3]
        R3_6 = R0_3.transpose()* Matrix(R0_g)*self.R_corr.transpose()
        R3_6 = R3_6.subs({th1: theta1, th2:theta2, th3: theta3})


        # Step 3: solve for theta4, theta5 and theta6
	# TODO: in the case of sin(theta5) > 0
	#       hint: make use of "tan2(,).evalf()" function and "R3_6[index]"
        theta4 = # something
        theta6 = # something
        theta5 = # something


	# TODO (optional):  in the case of sin(theta5) = 0 ?
	#		    in the case of sin(theta5) < 0 ?
	# (solve for theta4,5,6 in these two situations for extra credit)


	# Step 4: Sanity check
        def reduceEffectiveRange(theta, name):
            if(theta < -pi ):
                debug_log("        {} = {}, being changed to {}".format(name, theta, (theta + 2*pi)))
                return (theta + 2*pi)
            elif(theta > pi):
                debug_log("        {} = {}, being changed to {}".format(name, theta, (theta - 2*pi)))
                return (theta - 2*pi)
            else:
                return theta

        def angleCorrection(theta, old_theta):
            d = theta - old_theta
            if d > np.pi:
                return theta - 2*np.pi
            elif d < -np.pi:
                return theta + 2*np.pi
            else:
                return theta


        theta4 = reduceEffectiveRange(theta4, "theta4")
        theta6 = reduceEffectiveRange(theta6, "theta6")

        theta4 = angleCorrection(theta4, self.old_th4)
        theta6 = angleCorrection(theta6, self.old_th6)

        self.old_theta4 = theta4
        self.old_theta6 = theta6
        
        # Populate response for the IK request
	joint_trajectory_point = JointTrajectoryPoint()
        joint_trajectory_point.positions = (theta1, theta2, theta3, theta4, theta5, theta6)
        debug_log("    joint angles = {}".format(joint_trajectory_point.positions))
        return joint_trajectory_point

    def verifyIK(self, pose, joint_trajectory_point):

	debug_log("    verify IK result:")

	# check result in joint limit range
	theta = joint_trajectory_point.positions
        checkInRange(rtod(theta[0]), -185, 185, "theta1")
        checkInRange(rtod(theta[1]),  -45,  85, "theta2")
        checkInRange(rtod(theta[2]), -210,  65, "theta3")
        checkInRange(rtod(theta[3]), -350, 350, "theta4")
        checkInRange(rtod(theta[4]), -125, 125, "theta5")
        checkInRange(rtod(theta[5]), -350, 350, "theta6")

	# extract computed position
	result  = self.performFK([theta[0], theta[1], theta[2], theta[3], theta[4], theta[5]])
	computed_pos = Matrix([[result.position.x], [result.position.y], [result.position.z]])

        # extract requested position
        requested_pos = Matrix([[pose.position.x], [pose.position.y], [pose.position.z]])

	# error analysis
	error = (requested_pos - computed_pos).norm()

	# debug log
        debug_log("        EE position received FK: {}".format(requested_pos.tolist()))
        debug_log("        EE position after FK :   {}".format(computed_pos.tolist()))
        debug_log("        EE Error : {}".format(error))

	return error
