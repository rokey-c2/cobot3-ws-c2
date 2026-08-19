import math
import time
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped


def get_quaternion_from_euler(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]

def get_euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return roll, pitch, yaw

def print_final_pose(pose_msg):
    if not pose_msg:
        return
    pos = pose_msg.pose.position
    ori = pose_msg.pose.orientation
    roll_rad, pitch_rad, yaw_rad = get_euler_from_quaternion(ori.x, ori.y, ori.z, ori.w)
    yaw_deg = math.degrees(yaw_rad)
    
    print("-" * 50)
    print(f"📍 최종 위치: X = {pos.x:.3f} m, Y = {pos.y:.3f} m")
    print(f"🧭 최종 바라보는 방향 (Heading): {yaw_deg:.1f}°")
    print("-" * 50)

def create_pose(navigator, x, y, yaw_deg):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    q = get_quaternion_from_euler(0, 0, math.radians(yaw_deg))
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    return pose

# ==========================================================
# 🚀 메인 주행 로직 (다중 경유지 주행)
# ==========================================================
def main():
    rclpy.init()
    nav = BasicNavigator()
    
    # 1. 출발점 설정
    init_pose = create_pose(nav, 0.002, -0.024, 0.0)
    nav.setInitialPose(init_pose)
    nav.waitUntilNav2Active()
    
    # 2. 경유지(Waypoints) 리스트 생성
    waypoints = []
    
    # 예시: 로봇이 'ㄷ'자 형태로 이동하도록 설정
    waypoints.append(create_pose(nav, 1.0, 0.0, 90.0))   # 경유지 1
    waypoints.append(create_pose(nav, 2.0, -1.0, 90.0))  # 경유지 2
    waypoints.append(create_pose(nav, 1.5, 1.0 , 90.0))  # 경유지 3 (최종 목적지)
        
    # 3. Task 실행 (goToPose -> goThroughPoses로 변경)
    print("🚀 다중 경유지 주행을 시작합니다...")
    nav.goThroughPoses(waypoints)
    
    last_pose = None

    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        if feedback:
            last_pose = feedback.current_pose
            
            # 피드백에서 남은 총 거리와 남은 경유지 개수를 확인할 수 있습니다.
            print(f"남은 경유지 수: {feedback.number_of_poses_remaining} 개 | "
                  f"목적지까지 남은 총 거리: {feedback.distance_remaining:.2f} m")
            
        time.sleep(1.0)

    # 4. 결과 처리
    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        print('\n🎉 모든 경유지를 거쳐 목적지에 도착 완료!')
        print_final_pose(last_pose)
            
    elif result == TaskResult.CANCELED:
        print('\n⚠️ 주행이 취소되었습니다.')
    elif result == TaskResult.FAILED:
        print('\n❌ 주행 실패 (장애물 등으로 경로를 찾을 수 없음).')

    rclpy.shutdown()

if __name__ == '__main__':
    main()