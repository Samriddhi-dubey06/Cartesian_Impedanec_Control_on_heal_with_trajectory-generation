import rospy
import numpy as np
import PyKDL as kdl
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromParam
import tf.transformations as tf_trans
from scipy.spatial.transform import Slerp, Rotation

# ### IMPEDANCE GAINS ###
# K = np.diag([3250, 3250, 3490, 300, 300, 300])  # Lower stiffness  
# D = np.diag([70, 70, 90, 50, 50, 50])  # Higher damping  

### IMPEDANCE GAINS ###
K = np.diag([3210, 3210, 3420, 300, 300, 300])  # Lower stiffness  
D = np.diag([70, 70, 90, 40, 40, 40])  # Higher damping  

### TARGET POSES ###
# pose_1 = np.array([-0.0008958916792947791, 0.3687, 0.5513, -0.005875280975761145, -2.5560605096337954e-05, -3.578403838307283e-05])
# pose_2 = np.array([0.08263462381390414, 0.3466207085017308, 0.638477486817321, -0.8221743062798227, -1.4153906723737726, 0.5942672212831976])
# pose_1 = np.array([0.042403312135308, 0.43587539175727114, 0.36096462401206725,-1.4925528791472913, -0.7974286494256037, 1.265675007980416])
# pose_1 = np.array([0.07813767742546278, 0.34535888110223023, 0.7431219113967011, 0.9335448430258216, -1.122385486405234, -1.0232115999471527])
# pose_2 = np.array([0.04613301430626721, 0.46068137746629617, 0.18154952397633714, -1.620050475762634,  -0.7222691982259092, 1.4660447280821116])


# # lifting box 
# pose_2 = np.array([0.01998754783558579, 0.30608145527023173, 0.7511785464713486, 1.4052688490293344, -0.4877548890373451, -1.6918452657375265])
# pose_1 = np.array([-0.005187463885669041, 0.492870190829744, 0.45522837951943037, 1.7202684931707772,  -1.3941426719285053, -1.998549723626717])
# pose_3 =np.array([-0.1865387150663094,0.40487367459657103,  0.453532367824289, 1.3515434394440706,  -1.4840435634037261, -1.279515745484098])

# lifting box 
pose_2 = np.array([0.01998754783558579, 0.30608145527023173, 0.7511785464713486, 1.4052688490293344, -0.4877548890373451, -1.6918452657375265])
pose_1 = np.array([-0.005187463885669041, 0.30608145527023173, 0.49522837951943037, 1.7202684931707772,  -1.3941426719285053, -1.998549723626717])
pose_3 =np.array([-0.2265387150663094,0.30608145527023173,  0.493532367824289, 1.3515434394440706,  -1.4840435634037261, -1.279515745484098])


### CONTROL VARIABLES ###
current_pose = None
current_velocity = np.zeros(6)
current_joint_positions = None
current_joint_velocities = None

### ROS INIT ###
rospy.init_node('cartesian_impedance_control', anonymous=True)
robot = URDF.from_parameter_server()
ok, kdl_tree = treeFromParam('/robot_description')
if not ok:
    rospy.logerr("Failed to extract KDL tree from URDF")
    exit()

base_link = robot.get_root()
end_effector_link = "tool_ff"
kdl_chain = kdl_tree.getChain(base_link, end_effector_link)
kdl_jac_solver = kdl.ChainJntToJacSolver(kdl_chain)

### CALLBACKS ###
def end_effector_callback(msg):
    global current_pose
    position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
    orientation = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    euler_angles = tf_trans.euler_from_quaternion(orientation)
    current_pose = np.hstack((position, euler_angles))

def joint_state_callback(msg):
    global current_joint_positions, current_joint_velocities
    current_joint_positions = np.array(msg.position)
    current_joint_velocities = np.array(msg.velocity)

### TRAJECTORY PLANNING ###
def quintic_polynomial_trajectory(start, goal, t, T):
    """Computes a smooth quintic trajectory for position."""
    a0, a1, a2, a3, a4, a5 = start, 0, 0, 10*(goal-start)/T**3, -15*(goal-start)/T**4, 6*(goal-start)/T**5
    return a0 + a1*t + a2*t**2 + a3*t**3 + a4*t**4 + a5*t**5

def compute_slerp_orientation(start_euler, goal_euler, t, T):
    """Computes a smooth Slerp trajectory for orientation."""
    start_quat = tf_trans.quaternion_from_euler(*start_euler)
    goal_quat = tf_trans.quaternion_from_euler(*goal_euler)

    key_times = np.array([0, T])
    key_rots = Rotation.from_quat(np.array([start_quat, goal_quat]))
    slerp = Slerp(key_times, key_rots)
    
    return slerp(np.array([t])).as_quat()[0]

### COMPUTE FUNCTIONS ###
def compute_jacobian(joint_positions):
    kdl_joints = kdl.JntArray(len(joint_positions))
    for i in range(len(joint_positions)):
        kdl_joints[i] = joint_positions[i]
    jacobian = kdl.Jacobian(len(joint_positions))
    kdl_jac_solver.JntToJac(kdl_joints, jacobian)
    return np.array([[jacobian[i, j] for j in range(jacobian.columns())] for i in range(jacobian.rows())])

def compute_cartesian_impedance(target_pose, current_pose, current_velocity):
    position_error = target_pose[:3] - current_pose[:3]
    target_orientation = tf_trans.quaternion_from_euler(*target_pose[3:])
    current_orientation = tf_trans.quaternion_from_euler(*current_pose[3:])
    q_error = tf_trans.quaternion_multiply(target_orientation, tf_trans.quaternion_inverse(current_orientation))
    euler_error = np.array(tf_trans.euler_from_quaternion(q_error))
    velocity_error = -current_velocity
    force_torque = K @ np.hstack((position_error, euler_error)) + D @ velocity_error
    return force_torque

def execute_motion(target_pose):
    global current_pose
    rospy.Subscriber('/end_effector_pose', PoseStamped, end_effector_callback)
    rospy.Subscriber('/joint_states', JointState, joint_state_callback)
    torque_publisher = rospy.Publisher('/effort_controller/command', Float64MultiArray, queue_size=10)
    
    rate = rospy.Rate(500)
    while (current_pose is None or current_joint_positions is None) and not rospy.is_shutdown():
        rate.sleep()
    
    T = 15  
    num_steps = int(T * 500)  
    for i in range(num_steps + 1):
        if rospy.is_shutdown():
            break
        t = (i / num_steps) * T
        interpolated_position = np.array([quintic_polynomial_trajectory(current_pose[j], target_pose[j], t, T) for j in range(3)])
        interpolated_quaternion = compute_slerp_orientation(current_pose[3:], target_pose[3:], t, T)
        interpolated_euler = tf_trans.euler_from_quaternion(interpolated_quaternion)
        
        new_target_pose = np.hstack((interpolated_position, interpolated_euler))
        jacobian = compute_jacobian(current_joint_positions)
        end_effector_velocity = jacobian @ current_joint_velocities  
        force_torque = compute_cartesian_impedance(new_target_pose, current_pose, end_effector_velocity)
        
        lambda_damping = 0.4 
        J_dls = jacobian.T @ np.linalg.inv(jacobian @ jacobian.T + lambda_damping * np.eye(6))
        joint_torques = J_dls @ force_torque
        
        torque_msg = Float64MultiArray()
        torque_msg.data = joint_torques.tolist()
        torque_publisher.publish(torque_msg)
        rate.sleep()

    rospy.loginfo("Target reached. Holding position with impedance response.")
    
def maintain_impedance_control():
    """Keeps impedance control running indefinitely after reaching the final pose."""
    torque_publisher = rospy.Publisher('/effort_controller/command', Float64MultiArray, queue_size=10)
    rate = rospy.Rate(450)

    while not rospy.is_shutdown():
        if current_pose is not None and current_joint_positions is not None:
            jacobian = compute_jacobian(current_joint_positions)
            end_effector_velocity = jacobian @ current_joint_velocities  
            force_torque = compute_cartesian_impedance(pose_2, current_pose, end_effector_velocity)
            J_dls = jacobian.T @ np.linalg.inv(jacobian @ jacobian.T + 0.6 * np.eye(6))
            joint_torques = J_dls @ force_torque
            
            torque_msg = Float64MultiArray()
            torque_msg.data = joint_torques.tolist()
            torque_publisher.publish(torque_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.loginfo("Moving to first pose")
        execute_motion(pose_1)
        rospy.sleep(2)
        rospy.loginfo("Moving to second pose")
        execute_motion(pose_2)
        rospy.loginfo("Moving to second pose")
        rospy.loginfo("Maintaining impedance control indefinitely.")
        execute_motion(pose_3)
        rospy.loginfo("Moving to third pose")
        maintain_impedance_control()
    except rospy.ROSInterruptException:
        rospy.loginfo("Cartesian Impedance Control node terminated.")
