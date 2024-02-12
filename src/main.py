from BOT import BOT
from Frame_with_Aruco import DetectFrame
from threading import Thread
import time

def swarmnoid(marker):
    target_waste_ids = [9, 11, 13, 15, 17,8, 10, 12, 14, 16]
    robot_6_head_position, robot_6_center_position = get_head_position(6,marker.markers)
    robot_6 = BOT(6,robot_6_head_position,target_waste_ids)
    robot_7_head_position, robot_7_center_position = get_head_position(7,marker.markers)
    robot_7 = BOT(7,robot_7_head_position,target_waste_ids)
    while True:
        
        robot_6_head_position, robot_6_center_position = get_head_position(6,marker.markers)
        robot_7_head_position, robot_7_center_position = get_head_position(7,marker.markers)
        robot_6.update_centre_position(robot_6_center_position)
        robot_7.update_centre_position(robot_7_center_position)
        robot_6.update_position(robot_6_head_position)
        robot_7.update_position(robot_7_head_position)
        target_waste_ids.remove(robot_6.find_nearest_waste(marker.markers))
        target_waste_ids.remove(robot_7.find_nearest_waste(marker.markers))
        robot_6.plan_path()
        robot_7.plan_path(robot_6.path)
        robot_6.move_towards_goal()
        robot_7.move_towards_goal()
        
        
        
        
        


def get_head_position(robot_id, markers):
    """
    Return the head position, the top left and top right corner positions,
    and the center of the marker based on ArUco marker detection.
    """
    if robot_id in markers:
        for marker_data in markers[robot_id]:
            # Assuming corners are provided in the order: tl, tr, br, bl
            corners = marker_data["corners"]
            tl, tr, br, bl = corners[0], corners[1], corners[2], corners[3]

            # Calculate the midpoint between tl and tr for the head position
            head_position = (int((tl[0] + tr[0]) / 2), int((tl[1] + tr[1]) / 2))

            # Calculate the center of the marker as the average of all corners
            center_x = int((tl[0] + tr[0] + br[0] + bl[0]) / 4)
            center_y = int((tl[1] + tr[1] + br[1] + bl[1]) / 4)
            marker_center = (center_x, center_y)

            # # Ensure tl and tr are tuples of integers
            # tl = (int(tl[0]), int(tl[1]))
            # tr = (int(tr[0]), int(tr[1]))

            return head_position, marker_center
    return None, None




if __name__ == '__main__':
    marker = DetectFrame()
    # Process(target= fc.opencv).start()
    # Process(target=start(fc)).start()
    Thread(target= marker.capture_update_frame).start()
    time.sleep(2)
    Thread(target=swarmnoid(marker)).start()