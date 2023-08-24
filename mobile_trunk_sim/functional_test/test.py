from wheels_angles_compute import twistToWheelsAngularSpeed, move

elapsed_time = 0
deplacement_ctrl = 0
final_pos = 0

def test_move(angular_speed, linear_speed, duration, dt, robot_base_pos, WheelsMotors_angles_rest_position):
    print("dt = ", elapsed_time)
    elapsed_time +=dt
    elapsed_time +=dt
    deplacement = linear_speed * dt
    deplacement_ctrl +=deplacement

    if elapsed_time >= duration:
        angular_speed = 0
        linear_speed = 0
        final_pos =[robot_base_pos.value[0][0],
                        robot_base_pos.value[0][1],
                        robot_base_pos.value[0][2]]
        print("final position = ", final_pos, "traveled distance = ", deplacement_ctrl)
    
    else : 
        wheels_angular_speed = twistToWheelsAngularSpeed(angular_speed, linear_speed)
        move(WheelsMotors_angles_rest_position, wheels_angular_speed, dt)