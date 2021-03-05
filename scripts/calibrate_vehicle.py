#!/usr/bin/env python
import rospy
import time
import math
import numpy as np
from scipy.optimize import minimize
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray

num_rounds = 0
last_sample_time = 0.0
tests = ["Straight forward 5m", "CCW Rotation 180deg", "CW Rotation 180deg"]
actual_orientation = [0, 2*math.pi, 2*math.pi]
record = False

# Each entry in k_vals is [left_radius, right_radius, base_width]
k_vals = np.zeros([])
samples_arr = np.empty((0,3), float)

def turnObjFunc(x, *args):
    samples = args[0]
    actual_orientation = args[1]
    k1 = args[2]
    k2 = args[3]
    final_orientation = 0.0

    # Loop through all samples for this set and get final orientation
    # Sample = [v_left, v_right, time]
    for i in range(len(samples)):
        if i == len(samples) - 1:
            continue

        v_left = samples[i][0] * k1
        v_right = samples[i][1] * k2
        v_angular = (v_right - v_left) / x[0]

        # Forward simulate velocities to get final orientation
        final_orientation += v_angular * (samples[i+1][2] - samples[i][2])

    return abs(abs(final_orientation) - actual_orientation)

def straightObjFunc(x, *args):
    samples = args[0]
    actual_orientation = args[1]
    final_orientation = 0.0

    # Loop through all samples for this set and get final orientation
    # Sample = [v_left, v_right, time]
    for i in range(len(samples)):
        v_left = samples[i][0] * x[0]
        v_right = samples[i][1] * x[1]
        v_angular = (v_right - v_left) / x[2]

        # Forward simulate velocities to get final orientation
        if i > 0:
            final_orientation += v_angular * (samples[i][2] - samples[i-1][2])

    return abs(abs(final_orientation) - actual_orientation)

def velCB(array):
    global last_sample_time, samples_arr

    # Record data as long as record is true
    if record:
        if last_sample_time == 0.0:
            last_sample_time = time.time()
            return

        dt = time.time() - last_sample_time
        samples_arr = np.append(samples_arr, np.array([[array.data[0], array.data[1], dt]]), axis=0)
        #print("Appended", array.data)

    else:
        last_sample_time = 0.0


def calibrate_vehicle():
    rospy.Subscriber("/velocities_raw", Float32MultiArray, velCB)
    global num_rounds, samples_arr
    rospy.init_node('calibrate_vehicle')

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        if num_rounds > 0:
            # 3 test, cw rotation, ccw rotation, linear motions
            for j in range(num_rounds):
                # This rounds k values
                k_temp = [0,0,0]

                # Test number i
                for i in range(3):
                    # Clear recorded data
                    samples_arr = np.empty((0,3), float)

                    # Record
                    print("\n\nGet ready for test: " + tests[i] + " number " + str(j + 1))
                    raw_input("Press enter when ready")
                    global record
                    record = True

                    raw_input("Press enter again when task is complete")
                    record = False
                    print("Number of samples recorded: " + str(len(samples_arr)))

                    # Bounds for optimization, set accordingly
                    bnds_straight = ((0.5,1.5),(0.5,1.5),(0.5,1.5))
                    bnds_turn = ((0.4,1.5),)

                    # Optimize
                    # Straight line experiment
                    if i == 0:
                        res = minimize(straightObjFunc, (1,1,1), (samples_arr, actual_orientation[i]), method='SLSQP', bounds=bnds_straight)
                        print("\n\nOptimization results:")
                        print(res)
                        k_temp[0] = res.x[0]
                        k_temp[1] = res.x[1]

                    # Turn experiments
                    else:
                        res = minimize(turnObjFunc, (1), (samples_arr, actual_orientation[i], k_temp[0], k_temp[1]), method='SLSQP', bounds=bnds_turn)
                        print("\n\nOptimization results:")
                        print(res)
                        k_temp[2] += res.x[0]
                        
                k_vals[j][0] = k_temp[0]
                k_vals[j][1] = k_temp[1]

                # 2 turn experiments, get the average of both
                k_vals[j][2] = k_temp[2] / 2.0

                print(k_vals)

            # Calculate average for each parameter
            k_averaged = [0,0,0]
            for i in range(len(k_vals)):
                k_averaged[0] += k_vals[i][0]
                k_averaged[1] += k_vals[i][1]
                k_averaged[2] += k_vals[i][2]

            k_averaged[0] /= (len(k_vals))
            k_averaged[1] /= (len(k_vals))
            k_averaged[2] /= (len(k_vals))

            print("\n\n\n\nResults:")
            print("[left_wheel_radius, right_wheel_radius, base_width]")
            print(k_averaged)

            final_orientation = 0.0
            for i in range(len(samples_arr)):
                if i == len(samples_arr) - 1:
                    continue

                v_left = samples_arr[i][0]
                v_right = samples_arr[i][1]
                v_angular = (v_right - v_left) / 0.5

                # Forward simulate velocities to get final orientation
                final_orientation += v_angular * (samples_arr[i+1][2] - samples_arr[i][2])

            print(final_orientation)
            final_orientation = 0.0
            for i in range(len(samples_arr)):
                if i == len(samples_arr) - 1:
                    continue

                v_left = samples_arr[i][0] * k_averaged[0]
                v_right = samples_arr[i][1] * k_averaged[1]
                v_angular = (v_right - v_left) / k_averaged[2]

                # Forward simulate velocities to get final orientation
                final_orientation += v_angular * (samples_arr[i+1][2] - samples_arr[i][2])

            print(final_orientation)

            num_rounds = 0
            break

        rate.sleep()

if __name__ == '__main__':
    num_rounds = int(raw_input("How many sets of data samples (cw rotation, ccw rotation, 5m linear motion) to record for calibration? "))
    print("Recording " + str(num_rounds) + " data sets")
    k_vals = np.zeros((num_rounds, 3), float)

    try:
        calibrate_vehicle()

    except rospy.ROSInterruptException:
        pass