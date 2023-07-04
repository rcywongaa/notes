import numpy as np
from scipy.spatial.transform import Rotation
import quaternion

'''
This snippet compares different methods for calculating angular velocity from two quaternions
'''

# This is wrong!  Time derivative of euler angles is NOT angular velocity!
# def angular_velocity1(q1, q2):
#     return (q2 * q1.inv()).as_euler('zyx')

# From https://mariogc.com/post/angular-velocity-quaternions/
# uses scalar first format (w, x, y, z)
# This calculates angular velocity in BODY frame
# def angular_velocity2(q1, q2, dt=1):
#     return (2 / dt) * np.array([
#         q1[0]*q2[1] - q1[1]*q2[0] - q1[2]*q2[3] + q1[3]*q2[2],
#         q1[0]*q2[2] + q1[1]*q2[3] - q1[2]*q2[0] - q1[3]*q2[1],
#         q1[0]*q2[3] - q1[1]*q2[2] + q1[2]*q2[1] - q1[3]*q2[0]])

def angular_velocity3(q1, q2):
    return (q2 * q1.inv()).as_rotvec()

# From https://arxiv.org/pdf/0811.2889.pdf
# Eq. 16 (inertial frame)
def angular_velocity4(q1, q2):
    return 2*(np.array([
        [-q1.x, q1.w, -q1.z, q1.y],
        [-q1.y, q1.z, q1.w, -q1.x],
        [-q1.z, -q1.y, q1.z, q1.w]
    ])) @ quaternion.as_float_array(q2 - q1)

# From https://math.stackexchange.com/questions/160908/how-to-get-angular-velocity-from-difference-orientation-quaternion-and-time
def angular_velocity5(q1, q2):
    # return quaternion.as_float_array(2*(q2 - q1)/(q1))[1:]
    return quaternion.as_float_array(2*(np.log(q2/q1))/q1)[1:]

# From https://arxiv.org/pdf/0811.2889.pdf
# Eq. 17 (local frame)
# def angular_velocity6(q1, q2):
#    return 2*(np.array([
#       [-q1.x, q1.w, q1.z, -q1.y],
#       [-q1.y, -q1.z, q1.w, q1.x],
#       [-q1.z, q1.y, -q1.x, q1.w]
#    ])) @ quaternion.as_float_array(q2-q1)

def integrate(q1, vel):
  return q1_quat + 0.5*np.quaternion(0, vel[0], vel[1], vel[2])*q1_quat

if __name__ == "__main__":
  q1 = Rotation.from_euler('zyx', [10, 20, 30], degrees=True)
  # scipy uses scalar last format (x, y, z, w)
  q1_quat = q1.as_quat()
  # np.quaternion uses scalar first format (w, x, y, z)
  q1_quat = np.quaternion(q1_quat[3], q1_quat[0], q1_quat[1], q1_quat[2])
  q2 = Rotation.from_euler('zyx', [60, 50, 40], degrees=True)
  q2_quat = q2.as_quat()
  q2_quat = np.quaternion(q2_quat[3], q2_quat[0], q2_quat[1], q2_quat[2])

  print(f"Source quat q1: {q1_quat}, dest quat q2: {q2_quat}")

  # vel1 = angular_velocity1(q1, q2)
  # print(f"vel1 = {vel1}")
  # print(f"mag = {np.linalg.norm(vel1)}, direction = {vel1/np.linalg.norm(vel1)}")

  # vel2 = angular_velocity2(
  #    quaternion.as_float_array(q1_quat),
  #    quaternion.as_float_array(q2_quat)
  # )
  # print(f"vel2 = {vel2}")
  # print(f"mag = {np.linalg.norm(vel2)}, direction = {vel2/np.linalg.norm(vel2)}")

  vel3 = angular_velocity3(q1, q2)
  print(f"vel3 = {vel3}")
  print(f"mag = {np.linalg.norm(vel3)}, direction = {vel3/np.linalg.norm(vel3)}")
  print(f"Projected q2 = {integrate(q1, vel3)}")

  vel4 = angular_velocity4(q1_quat, q2_quat)
  print(f"vel4 = {vel4}")
  print(f"mag = {np.linalg.norm(vel4)}, direction = {vel4/np.linalg.norm(vel4)}")
  print(f"Projected q2 = {integrate(q1, vel4)}")

  vel5 = angular_velocity5(q1_quat, q2_quat)
  print(f"vel5 = {vel5}")
  print(f"mag = {np.linalg.norm(vel5)}, direction = {vel5/np.linalg.norm(vel5)}")
  print(f"Projected q2 = {integrate(q1, vel5)}")

  # vel6 = angular_velocity6(q1_quat, q2_quat)
  # print(f"vel6 = {vel6}")
  # print(f"mag = {np.linalg.norm(vel6)}, direction = {vel6/np.linalg.norm(vel6)}")
  import pdb; pdb.set_trace()
