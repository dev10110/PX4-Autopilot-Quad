# QuadControlLog msg 
# defines the inputs and outputs of QuadControl 
# values should all be in NED frame

uint64 timestamp # time since system start (microseconds)

# setpoints 
float32[3] x_ref
float32[3] v_ref
float32[3] a_ref
float32[3] j_ref
float32[3] s_ref # snap reference
float32 yaw_ref
float32 yaw_vel_ref
float32 yaw_acc_ref
float32[3] b1_ref
float32[3] omega_ref # refers to Omega_ref in the code
float32[3] alpha_ref

# current state
float32[3] x 
float32[3] v
float32[3] b1 # rotation matrix
float32[3] b2
float32[3] b3
float32[3] omega # refers to Omega in the code

# internal vars
float32[3] ex
float32[3] ev
float32[3] ei
float32[3] thrust
float32[3] b1d # desired rotation matrix
float32[3] b2d
float32[3] b3d
float32[3] er # refers to eR in the code
float32[3] eomega # refers to eOmega in the code

# final commands
float32 thrust_cmd
float32[3] torque_cmd
