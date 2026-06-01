import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO
import math
import serial
import time

# to convert metrics for testing
MTI = 39.37

# meters per bit for hex conversion
DIST_MPB = 0.00245
LR_MPB = 0.0196
UD_MPB = 0.0625
YAW_MPB = 0.625

def main():
    # ser = serial.Serial('COM3', 9600, timeout=1) # open when serial is needed
    time.sleep(2)

    pipeline = rs.pipeline()
    config = rs.config()

    # streaming settings
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    pipeline.start(config)

    # select machine learning model to use
    model = YOLO("BOVKeypointModel2.pt")

    # aligns color and depth streams
    align = rs.align(rs.stream.color)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()

            if not color_frame or not depth_frame:
                continue
            
            color_image = np.asanyarray(color_frame.get_data())
            results = model(color_image)

            # define array of keypoint coordinates in resolution coords
            points_res = [[-1, -1] for _ in range(4)]

            # define array of keypoint coordinates in 3D
            points = [[-1, -1, -1] for _ in range(4)]

            # extract data from keypoints and translate into 3D coordinates
            for result in results:
                # kps = result.keypoints.xy
                # confs = result.keypoints.conf
                boxes = result.boxes
                for box in boxes:
                    box_conf = box.conf
                    if box_conf <= 0.7:
                        continue
                    print(f"back detected")
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    for kps in result.keypoints.xy:
                        confs = result.keypoints.conf
                        for i in range(4):
                            if confs[0][i] <= 0.7:
                                continue
                            x = int(kps[i][0])
                            y = int(kps[i][1])
                            
                            # convert to 3D coordinates
                            if x == 640:
                                x -= 1
                            if y == 480:
                                y -= 1
                            depth = depth_frame.get_distance(x, y)
                            depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
                            point = rs.rs2_deproject_pixel_to_point(depth_intrin, [x, y], depth)
                            
                            # 0 = tr, 1 = br, 2 = bl, 3 = tl
                            cv2.circle(color_image, (x, y), 2, (0, 0, 255), 1)

                            # add into arrays
                            points_res[i] = [x, y]
                            points[i] = point
                            print(f"Keypoint {i} depth: {depth}, coords: {points[i]}")
            
            # use 3D coordinates to find direction/angle of vehicle
            # first check if the points exist (need 3 existing)
            n = calculateNormal(points)
            if n[0] == -1:
                print(f"3 points could not be identified\n")
            else:
                yaw = getYaw(n)
                p1 = int((points_res[0][0] + points_res[3][0]) / 2)
                p2 = int((points_res[0][1] + points_res[1][1]) / 2)
                z = depth_frame.get_distance(p1, p2)
                cv2.circle(color_image, (p1, p1), 2, (255, 0, 0), 1)
                depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
                point = rs.rs2_deproject_pixel_to_point(depth_intrin, [x, y], z)
                print(f"X: {round(point[0] * MTI, 2)}, Y: {round(point[1] * MTI, 2)}, Z: {round(point[2] * MTI, 2)}, Yaw: {round(yaw, 2)}")

                # format data for CAN bus (x = back and forth, y = up and down, z = left and right)
                data = formatData(x, y, z, yaw)

                # ser.write(data)
                print(f"Data in hex: {data}\n")

            cv2.imshow('RBG Stream', color_image)
            key = cv2.waitKey(1)
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break
    finally:
        # ser.close()
        pipeline.stop()

def calculateNormal(points):
    # check if there are at least 3 valid points
    i = 0
    for point in points:
        if point[0] != -1:
            if i == 0:
                p1 = point
            if i == 1:
                p2 = point
            if i == 2:
                p3 = point
            i += 1
    
    if i < 3:
        return [-1, -1, -1]

    # calculate normal vector
    u = [p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]]
    v = [p3[0] - p1[0], p3[1] - p1[1], p3[2] - p1[2]]
    n = [u[1]*v[2] - u[2]*v[1], u[2]*v[0] - u[0]*v[2], u[0]*v[1] - u[1]*v[0]]
    if n[2] < 0:
        n[2] = -n[2]

    return n

def getYaw(n):
    yaw_rad = math.atan2(n[2], n[0])
    yaw_deg = math.degrees(yaw_rad)
    return yaw_deg #- 90 # ?

# format data into CAN bus hex number 0x370 00(x) 0(y) 000(z) 00(yaw angle)
# for distances up to ~10m (0.00245m per bit for 4096), left/right (-2.5m to 2.5m, 0.0196m per bit for 256)
# up/down (-0.5m to 0.5m, 0.0625m per bit for 16), angle (-80 to 80 degrees, 0.625 degrees per bit for 256)
def formatData(x, y, z, yaw):
    # format distance (depth)
    z_hex = hex(round(z / DIST_MPB))

    # format left/right, can be negative so normalize: 0 = 128
    if abs(x) >= (256 * LR_MPB / 2):
        if x < 0:
            x_hex = hex(0)
        else:
            x_hex = hex(255)
    else:
        if x < 0:
            x_hex = hex(128 + round(x / LR_MPB))
        else:
            x_hex = hex(round(x / LR_MPB) + 128)

    #format up/down (0 = 8)
    if abs(y) >= (16 * UD_MPB / 2):
        if y < 0:
            y_hex = hex(0)
        else:
            y_hex = hex(15)
    else:
        if y < 0:
            y_hex = hex(8 + round(y / UD_MPB))
        else:
            y_hex = hex(round(y / UD_MPB) + 8)

    #format yaw angle (0 = 128)
    if abs(yaw) >= 2.5088:
        if yaw < 0:
            yaw_hex = hex(0)
        else:
            yaw_hex = hex(255)
    else:
        if yaw < 0:
            yaw_hex = hex(128 + round(yaw / YAW_MPB))
        else:
            yaw_hex = hex(round(yaw / YAW_MPB) + 128)

    return "370" + x_hex.removeprefix("0x") + y_hex.removeprefix("0x") + z_hex.removeprefix("0x") + yaw_hex.removeprefix("0x")

main()