import csv
import matplotlib.pyplot as plt
import numpy as np

class enumRadarColumns:
    sensor_type = 0
    rho_measured = 1
    phi_measured = 2
    rhodot_measured = 3
    timestamp = 4
    x_groundtruth = 5
    y_groundtruth = 6
    vx_groundtruth =7
    vy_groundtruth =8
    yaw_groundtruth = 9
    yawrate_groundtruth =10

class enumLidarColumns:
    sensor_type = 0
    x_measured = 1
    y_measured = 2
    timestamp = 3
    x_groundtruth = 4
    y_groundtruth = 5
    vx_groundtruth = 6
    vy_groundtruth = 7
    yaw_groundtruth = 8
    yawrate_groundtruth =9

a_list = []
yaw_dd_list = []
v_list = []
yaw_list = []
yaw_d_list = []
original_t = 0
original_v = 0
original_yawRate = 0
with  open('obj_pose-laser-radar-synthetic-input.txt', 'r') as csvfile:


    reader = csv.reader(csvfile, delimiter='\t')
    for row in reader:
        sensor_type = row[enumRadarColumns.sensor_type]

        if (sensor_type == "R"):
            time_s = float(row[enumRadarColumns.timestamp]) / 1.0e6
            vx = float(row[enumRadarColumns.vx_groundtruth])
            vy = float(row[enumRadarColumns.vy_groundtruth])
            yawRate = float(row[enumRadarColumns.yawrate_groundtruth])
            yaw = float(row[enumRadarColumns.yaw_groundtruth])
        else:
            time_s = float(row[enumLidarColumns.timestamp]) / 1.0e6
            vx = float(row[enumLidarColumns.vx_groundtruth])
            vy = float(row[enumLidarColumns.vy_groundtruth])
            yawRate = float(row[enumLidarColumns.yawrate_groundtruth])
            yaw = float(row[enumRadarColumns.yaw_groundtruth])

        v = pow(vx*vx + vy*vy, 0.5)
        if original_t == 0:
            original_t = time_s
            original_v = v
            original_yawRate = yawRate
        else:
            dt = time_s - original_t
            a = (v - original_v)/ dt
            yaw_dd = (yawRate - original_yawRate)/dt

            a_list.append(a)
            yaw_dd_list.append(yaw_dd)
            v_list.append(v)
            yaw_list.append(yaw)
            yaw_d_list.append(yawRate)

            original_t = time_s
            original_v = v
            original_yawRate = yawRate

a_list_np = np.array(a_list)
yaw_dd_list_np = np.array(yaw_dd_list)


print(" a standard deviation:")
print(np.std(a_list_np))
print(" yaw_dd standard deviation:")
print(np.std(yaw_dd_list_np))

plt.figure()
plt.plot(a_list)
plt.title('acceleration')

plt.figure()
plt.plot(yaw_dd_list)
plt.title('yaw acceleration')

plt.figure()
plt.plot(v_list)
plt.title('velocity')

plt.figure()
plt.plot(yaw_list)
plt.title('yaw')

plt.figure()
plt.plot(yaw_d_list)
plt.title('yaw rate')