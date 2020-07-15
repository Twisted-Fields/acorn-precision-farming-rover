
import math

steering_track = 1.5
wheel_base_ = 1.830
wheel_radius_ = 0.4
M_PI_2 = math.pi/2.0


# Both should be from 0-1.
def calculate_steering(steer, throttle):

    throttle_cmd = throttle
    angle_cmd = steer * math.radians(45)

    front_left_steering = 0
    front_right_steering = 0
    rear_left_steering = 0
    rear_right_steering = 0
    vel_left_front = 0
    vel_right_front = 0
    vel_right_rear = 0
    vel_left_rear = 0

    #// Compute wheels velocities:
    if(math.fabs(throttle_cmd) > 0.001):

      vel_steering_offset = 0#(angle_cmd*wheel_steering_y_offset_)/wheel_radius_
      vel_left_front  = throttle_cmd * math.sqrt((math.pow(throttle_cmd - angle_cmd*steering_track/2,2)
                                                                         +math.pow(wheel_base_*angle_cmd/2.0,2)))/wheel_radius_ - vel_steering_offset
      vel_right_front = throttle_cmd * math.sqrt((math.pow(throttle_cmd + angle_cmd*steering_track/2,2)
                                                                         +math.pow(wheel_base_*angle_cmd/2.0,2)))/wheel_radius_ + vel_steering_offset
      vel_left_rear = throttle_cmd * math.sqrt((math.pow(throttle_cmd - angle_cmd*steering_track/2,2)
                                                                       +math.pow(wheel_base_*angle_cmd/2.0,2)))/wheel_radius_ - vel_steering_offset
      vel_right_rear = throttle_cmd * math.sqrt((math.pow(throttle_cmd + angle_cmd*steering_track/2,2)
                                                                        +math.pow(wheel_base_*angle_cmd/2.0,2)))/wheel_radius_ + vel_steering_offset

    # #// // Compute steering angles
    throttle_cmd = 1.0
    if math.fabs(2.0*throttle_cmd) > math.fabs(angle_cmd*steering_track):
      front_left_steering = math.atan(angle_cmd*wheel_base_ /
                                  (2.0*throttle_cmd + angle_cmd*steering_track))
      front_right_steering = math.atan(angle_cmd*wheel_base_ /
                                   (2.0*throttle_cmd - angle_cmd*steering_track))
      rear_left_steering = -math.atan(angle_cmd*wheel_base_ /
                                  (2.0*throttle_cmd + angle_cmd*steering_track))
      rear_right_steering = -math.atan(angle_cmd*wheel_base_ /
                                   (2.0*throttle_cmd - angle_cmd*steering_track))


    return {"front_left":(front_left_steering, vel_left_front), "front_right": (front_right_steering, vel_right_front), "rear_left":(rear_left_steering, vel_left_rear), "rear_right": (rear_right_steering, vel_right_rear)}


def normalize_values(angle, throttle):
    if angle > 90:
        angle -= 180
        throttle *= -1
    if angle < -90:
        angle += 180
        throttle *= -1
    return angle, throttle

def calculate_steering2(steer, throttle, strafe):

    #print("steer {}, throttle {}".format(steer, throttle))

    #strafe = 0

    L = wheel_base_
    W = steering_track
    R = math.sqrt(L * L + W * W);

    steer = steer * throttle

    throttle_factor = 2.5


    RCW = steer
    FWD = throttle
    STR = strafe

    A = STR - RCW * (L/R)
    B = STR + RCW * (L/R)
    C = FWD - RCW * (W/R)
    D = FWD + RCW * (W/R)


    ws1 = math.sqrt(B * B + C * C) * throttle_factor
    ws2 = math.sqrt(B * B + D * D) * throttle_factor
    ws3 = math.sqrt(A * A + D * D) * throttle_factor
    ws4 = math.sqrt(A * A + C * C) * throttle_factor

    wa1 = math.atan2(B,C)*180.0/math.pi
    wa2 = math.atan2(B,D)*180.0/math.pi
    wa3 = math.atan2(A,D)*180.0/math.pi
    wa4 = math.atan2(A,C)*180.0/math.pi


    wa1, ws1 = normalize_values(wa1, ws1)
    wa2, ws2 = normalize_values(wa2, ws2)
    wa3, ws3 = normalize_values(wa3, ws3)
    wa4, ws4 = normalize_values(wa4, ws4)


    front_left_steering = math.radians(wa2)
    front_right_steering = math.radians(wa1)
    rear_right_steering = math.radians(wa4)
    rear_left_steering = math.radians(wa3)
    vel_left_front = ws2
    vel_right_front = ws1
    vel_right_rear = ws4
    vel_left_rear = ws3

    return {"front_left":(front_left_steering, vel_left_front), "front_right": (front_right_steering, vel_right_front), "rear_left":(rear_left_steering, vel_left_rear), "rear_right": (rear_right_steering, vel_right_rear)}
