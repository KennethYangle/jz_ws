import json
import numpy as np
import time
import airsim
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
from recursive_hungarian import RHA2
from freeflight import FreeFlight


class Allocation:
    def __init__(self):
        self.nums = 1
        self.width = 640
        self.height = 480
        self.home = []
        self.low = np.array([[109, 27, 100], [174, 205, 50]])
        self.high = np.array([[112, 40, 145], [177, 255, 110]])
        self.targets = len(self.low)
        self.th_Sam = 1e-4  # 太小会多判出目标；太大会合并目标，最好要0号飞机看到所有目标，能保证的话取大一些更好
        self.velocity = []
        self.is_reallocation = True
        self.config = {}
        self.finished = set()
        self.stop_id = -1


    def calc_centroid(self, image_bgr, i):
        min_prop = 0.00001
        max_prop = 0.1

        image_hue = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
        cent = []
        for t in range(self.targets):
            if t in self.finished:
                cent.append([-1, -1])
                continue

            th = cv2.inRange(image_hue, self.low[t], self.high[t])
            # dilated = cv2.dilate(th,
            #                     cv2.getStructuringElement(
            #                         cv2.MORPH_ELLIPSE, (3, 3)),
            #                     iterations=1)
            dilated = cv2.medianBlur(th, 3)
            # if t == 0:
            #     cv2.imshow("Dilated{}".format(i), dilated)

            M = cv2.moments(dilated, binaryImage=True)
            if M["m00"] >= min_prop * self.height * self.width:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                cent.append([cx, cy]) 
            else:
                cent.append([-1, -1])

            if M["m00"] >= max_prop * self.height * self.width:
                self.is_reallocation  = True
                self.finished.add(t)
                self.stop_id = i

        return cent


    def Sampson(self, R0, R1, T0, T1, pp0, pp1, K):
        def skew(v):
            return np.array([[0, -v[2,0], v[1,0]], 
                             [v[2,0], 0, -v[0,0]], 
                             [-v[1,0], v[0,0], 0]])

        R_c1c0 = R0.dot(R1.T)
        T_c1c0 = R0.dot(T1 - T0)
        F = np.dot(skew(T_c1c0), R_c1c0)
        pi0 = np.array(pp0+[1]).reshape((-1,1))
        pi1 = np.array(pp1+[1]).reshape((-1,1))
        # print(pi0, pi1)
        p0 = np.linalg.inv(K).dot(pi0)
        p1 = np.linalg.inv(K).dot(pi1)

        num = (p0.T.dot(F).dot(p1))[0,0] ** 2
        a = p0.T.dot(F)
        b = F.dot(p1)
        den = a[0,0]**2 + a[0,1]**2 + b[0,0]**2 + b[1,0]**2
        return num/den


    def match_feature(self, stash_feature, R_ec, T_ce, K):
        features = []
        for f in stash_feature[0]:  # 初始化feature
            if f != [-1,-1]:
                features.append([f])
        for i in range(1, len(stash_feature)):  # 对于剩下的飞机
            for j in range(len(stash_feature[i])):  # 每个飞机看到的特征
                if stash_feature[i][j] == [-1, -1]: # 没看到跳过
                    continue
                store = []
                for k in range(len(features)):       # 去已匹配的里面找
                    for l in range(len(features[k])):    # 第k个目标找到第l架飞机确实看到
                        if features[k][l] != [-1,-1]:
                            break
                    if features[k][l] == [-1,-1]:
                        store.append(1e5)
                        continue
                    store.append(self.Sampson(R_ec[l], R_ec[i], T_ce[l], T_ce[i], features[k][l], stash_feature[i][j], K))
                    # print(store[-1])     # 第i架飞机和第l架飞机，第k个特征与第j个特征辛普森距离
                minSam = min(store)
                minidx = store.index(minSam)
                if minSam < self.th_Sam:
                    features[minidx].append(stash_feature[i][j])
                else:
                    features.append([[-1, -1] for s in range(i)])
                    features[-1].append(stash_feature[i][j])
            for k in range(len(features)):
                if len(features[k]) < i+1:
                    features[k].extend([[-1,-1] for s in range(i+1-len(features[k]))])
        print("features: {}".format(features))


    def reconstruction(self, feature, pose, angle) -> np.array:
        def quaternion2rotation(q):
            w, x, y, z = q[0], q[1], q[2], q[3]
            return np.array([[1-2*y*y-2*z*z, 2*x*y-2*w*z, 2*x*z+2*w*y],
                             [2*x*y+2*w*z, 1-2*x*x-2*z*z, 2*y*z-2*w*x],
                             [2*x*z-2*w*y, 2*y*z+2*w*x, 1-2*x*x-2*y*y]])

        f, u0, v0 = self.width/2, self.width/2, self.height/2
        K = np.array([[f,0,u0], [0,f,v0], [0,0,1]])
        R_bc = np.array([[0,1,0], [0,0,1], [1,0,0]])

        MM = []
        R_ec, T_ce = [], []
        for i in range(self.nums):
            vehicle_name = "Drone{}".format(i)
            R_be = quaternion2rotation(angle[i])
            R_ec.append( np.dot(R_bc, R_be.T) )
            T_ce.append( np.array(pose[i]).reshape((-1,1)) )
            MM.append( K.dot(R_ec[i]).dot(np.hstack((np.identity(3),-T_ce[i]))) )
            print("[{}]: R_ec: {}, T_ce: {}".format(vehicle_name, R_ec[i], T_ce[i]))

        # features = self.match_feature(feature, R_ec, T_ce, K)

        target_pos = np.zeros((1,3))
        for t in range(self.targets):
            srcA, srcb = [], []
            for i in range(self.nums):
                if feature[i][t] == [-1, -1]:
                    continue
                xi, yi = feature[i][t][0], feature[i][t][1]
                M = MM[i]
                srcA.append([M[2,0]*xi-M[0,0], M[2,1]*xi-M[0,1], M[2,2]*xi-M[0,2]])
                srcA.append([M[2,0]*yi-M[1,0], M[2,1]*yi-M[1,1], M[2,2]*yi-M[1,2]])
                srcb.append([M[0,3]-M[2,3]*xi])
                srcb.append([M[1,3]-M[2,3]*yi])

            if len(srcA) >= 4:
                ret, dstP = cv2.solve(np.array(srcA), np.array(srcb), flags=cv2.DECOMP_SVD)
                if not ret:
                    print("Solve Failed!!!")
                target_pos = np.vstack((target_pos, dstP.reshape((1,-1))))
            else:
                target_pos = np.vstack((target_pos, np.array([-1,-1,-1])))
        return target_pos[1:]


    def draw_reticle(self, image, feature):
        x, y = feature[0], feature[1]
        s = 10
        cv2.rectangle(image, (x-s, y-s), (x+s, y+s), (0, 0, 255), 2)
        cv2.line(image, (x, y-2*s), (x, y+2*s), (0, 0, 255), 2)
        cv2.line(image, (x-2*s, y), (x+2*s, y), (0, 0, 255), 2)


    def interception(self, feature, velocity, angle):
        def quaternion2yaw(q):
            w, x, y, z = q[0], q[1], q[2], q[3]
            return np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        kx, kz = 0.1, 0.01
        ex, ey = feature[0] - self.width/2, feature[1] - self.height/2
        yaw = quaternion2yaw(angle)
        return velocity*np.cos(yaw), \
               velocity*np.sin(yaw), \
               kz*ey, \
               kx*ex


    def map_and_allocation(self, stash_feature, stash_pose, stash_angle):
        target_pos = self.reconstruction(stash_feature, stash_pose, stash_angle)
        print("target pose: {}".format(target_pos))

        r = RHA2(stash_pose, target_pos)
        task = r.deal()
        print("task: {}".format(task[1]))

        for i in range(self.nums):
            vehicle_name = "Drone{}".format(i)
            target_pos = np.array(target_pos).tolist()
            idx = target_pos.index(task[1][i].tolist())
            self.config[vehicle_name] = idx


    def main(self):
        settings_file = open("/home/zhenglong/Documents/AirSim/settings.json")
        settings = json.load(settings_file)
        self.nums = len(settings["Vehicles"])
        self.height = settings["CameraDefaults"]["CaptureSettings"][0]["Height"]
        self.width = settings["CameraDefaults"]["CaptureSettings"][0]["Width"]
        print("The num of drones: {}".format(self.nums))
        for i in range(self.nums):
            vehicle_name = "Drone{}".format(i)
            self.home.append([settings["Vehicles"][vehicle_name]["X"],
                              settings["Vehicles"][vehicle_name]["Y"],
                              settings["Vehicles"][vehicle_name]["Z"]])

        ff = FreeFlight(self.home, [[0.0, 0.0, 0.0] for i in range(self.nums)])

        client = airsim.MultirotorClient()
        client.confirmConnection()

        for i in range(self.nums):
            self.velocity.append(i*0.5+1)

        for i in range(self.nums):
            vehicle_name = "Drone{}".format(i)
            client.enableApiControl(True, vehicle_name=vehicle_name)
            client.armDisarm(True, vehicle_name=vehicle_name)
            client.takeoffAsync(vehicle_name=vehicle_name)

        while True:
            stash_pose = []
            stash_angle = []
            stash_vel = []
            stash_feature = []
            image_bgr = []

            for i in range(self.nums):
                vehicle_name = "Drone{}".format(i)
                responses = client.simGetImages([
                    airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
                ],
                                                vehicle_name=vehicle_name)
                response = responses[0]
                if response is None:
                    print(
                        "Camera is not returning image, please check airsim for error messages"
                    )
                    sys.exit(0)
                else:
                    img1d = np.fromstring(response.image_data_uint8,
                                        dtype=np.uint8)
                    image_rgba = img1d.reshape(self.height, self.width, 4)
                    image_bgr.append( cv2.cvtColor(image_rgba, cv2.COLOR_RGBA2BGR) )

                cent = self.calc_centroid(image_bgr[i], i)
                stash_feature.append(cent)
                print("[{}]: feature: {}".format(vehicle_name, stash_feature[i]))

                kinematics = client.simGetGroundTruthKinematics(
                    vehicle_name=vehicle_name)
                stash_pose.append([
                    kinematics.position.x_val + self.home[i][0], 
                    kinematics.position.y_val + self.home[i][1],
                    kinematics.position.z_val + self.home[i][2]
                ])
                stash_vel.append([
                    kinematics.linear_velocity.x_val,
                    kinematics.linear_velocity.y_val,
                    kinematics.linear_velocity.z_val
                ])
                stash_angle.append([
                    kinematics.orientation.w_val, kinematics.orientation.x_val,
                    kinematics.orientation.y_val, kinematics.orientation.z_val
                ])
                print("[{}]: pose: {}, angle: {}".format(vehicle_name,
                                                        stash_pose[i],
                                                        stash_angle[i]))

                # client.moveByVelocityAsync(
                #     vx=0,
                #     vy=1,
                #     vz=0,
                #     duration=1,
                #     drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                #     vehicle_name=vehicle_name)

            ff.update(stash_pose, stash_vel)
            ff.controller()

            stash_pose = np.array(stash_pose)
            stash_angle = np.array(stash_angle)

            if self.is_reallocation:
                if len(self.finished) == self.targets:
                    print("task finished!")
                    break
                self.is_reallocation = False
                self.map_and_allocation(stash_feature, stash_pose, stash_angle)
            print("config: {}".format(self.config))

            for i in range(self.nums):
                vehicle_name = "Drone{}".format(i)
                idx = self.config[vehicle_name]

                self.draw_reticle(image_bgr[i], stash_feature[i][idx])
                cv2.imshow("Image{}".format(i), image_bgr[i])

                if stash_feature[i][idx] == [-1,-1] or i == self.stop_id:
                    self.is_reallocation = True
                    self.stop_id = -1
                    client.moveByVelocityAsync(0, 0, 0, 1, vehicle_name = vehicle_name, 
                                               drivetrain = airsim.DrivetrainType.MaxDegreeOfFreedom, 
                                               yaw_mode = airsim.YawMode(True, -30))
                else:
                    vx, vy, vz, yawrate = self.interception(stash_feature[i][idx], self.velocity[i], stash_angle[i])
                    vrx, vry, vrz = ff.swarm[i].repulsion
                    print("UAV{} repulsion: {}".format(i, ff.swarm[i].repulsion))
                    client.moveByVelocityAsync(vx+vrx, vy+vry, vz+vrz, 1, vehicle_name = vehicle_name, 
                                               drivetrain = airsim.DrivetrainType.MaxDegreeOfFreedom, 
                                               yaw_mode = airsim.YawMode(True, yawrate))

            key = cv2.waitKey(1) & 0xFF
            if (key == 27 or key == ord('q') or key == ord('x')):
                break

        client.reset()


if __name__ == "__main__":
    a = Allocation()
    a.main()