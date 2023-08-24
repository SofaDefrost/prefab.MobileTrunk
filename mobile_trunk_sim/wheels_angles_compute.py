from geometry_msgs.msg import  Pose2D
import math

#wheel_ray = 0.117 # rayon de la roue
#lz = 0.229 #distance entre le centre d'une des roues et le centre du chassis sur l'axe des z (458/2)
#lx = 0.2355 #distance entre le centre d'une des roues et le centre du chassis sur l'axe des x(142/2 + (614-(2*142))/2)

wheel_base_ = 0.569 #distance between front and rear axles
track_width_ = 0.543 #distance between right and left wheels
wheel_diameter_ = 0.233 #wheel diamater, to convert from angular speed to linear
wheels_max_speed = 27.27 # rad/s
wheels_angular_speed = [0, 0, 0, 0]

def twistToWheelsAngularSpeed(wy, vz):
    """
        The robot moves along the z axis and rotates on the y axis.
        twistToWheelsAngles method calculate the angular velocity
            of each of the wheels

    """
    lab = wheel_base_ / 2.0 + track_width_ / 2.0

    frw2 =  (vz + lab * wy) / (wheel_diameter_ / 2.0) # roue avant droite
    flw1 =  (vz - lab * wy) / (wheel_diameter_ / 2.0) # roue avant gauche
    blw3 =  (vz - lab * wy) / (wheel_diameter_ / 2.0) # roue arriere gauche
    brw4 = (vz + lab * wy) / (wheel_diameter_ / 2.0)  #roue arrière droite

    wheels_angular_speed = [flw1 , frw2 , blw3, brw4]
    
    wheels_angular_speed = setJointVelocityReferenceBetweenLimits(wheels_angular_speed)

    return wheels_angular_speed

def setJointVelocityReferenceBetweenLimits(wheels_angular_speed):
    """
      
    """
    max_scale_factor = 1.0
    lower_limit = - wheels_max_speed 
    upper_limit = wheels_max_speed
    lower_scale_factor =  upper_scale_factor = 1.0
    for i in range(0, 4):

        if wheels_angular_speed[i] < lower_limit:
            lower_scale_factor = abs(wheels_angular_speed[i] / lower_limit)

        if wheels_angular_speed[i] < upper_limit:
            upper_scale_factor = abs(wheels_angular_speed[i] / upper_limit)

        max_scale_factor = max(max_scale_factor, max(lower_scale_factor, upper_scale_factor))

    for i in range(0, 4):
        wheels_angular_speed[i] /= max_scale_factor
    i = i+1
    return wheels_angular_speed

def updateOdometry(dt):
    vx = 0.0
    w = 0.0
    vz = 0.0
    robot_pose_ = Pose2D()
    frw1 = wheels_angular_speed[0]
    flw2 = wheels_angular_speed[1]
    blw3 = wheels_angular_speed[2]
    brw4 = wheels_angular_speed[3]

    if (abs(frw1) < 0.001) : frw1 = 0.0
    if (abs(flw2) < 0.001) : flw2 = 0.0
    if (abs(blw3) < 0.001) : blw3 = 0.0
    if (abs(brw4) < 0.001) : brw4 = 0.0

    v_left_mps = ((flw2 + blw3) / 2.0)
    v_right_mps = ((frw1 + brw4) / 2.0)

    fDistanceBetweenWheels = 0.92
    vz = (v_right_mps + v_left_mps) / 2.0 #m/s
    vx = 0.0
    w = (v_right_mps - v_left_mps) / fDistanceBetweenWheels; # rad/s
    robot_pose_.x += math.cos(robot_pose_.theta) * vz * dt- math.sin(robot_pose_.theta) * vx * dt
    robot_pose_.y += math.sin(robot_pose_.theta) * vz * dt+ math.cos(robot_pose_.theta) * vx * dt
    # qt = Quaternion()
    # qt.quaternion_from_euler(0,0, robot_pose_.theta)
    # vt =Vector3(robot_pose_.x , robot_pose_.y, 0)
    return robot_pose_

def move(WheelsMotors_angles_rest_position, wheels_angular_speed, dt):
    """
    Move method allow the robot to move thanks to the angular velocity of 
    each of the wheels
    """
    with WheelsMotors_angles_rest_position.writeable() as angles:
        #Make the wheel turn according to their angular speed
        angles[0] += wheels_angular_speed[0] * dt
        angles[1] += wheels_angular_speed[1] * dt
        angles[2] += wheels_angular_speed[2] * dt
        angles[3] += wheels_angular_speed[3] * dt
    #print("------------>", WheelsMotors_angles_rest_position.value)
    return WheelsMotors_angles_rest_position.value