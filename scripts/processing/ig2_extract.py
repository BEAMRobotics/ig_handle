import rosbag
import sys
import argparse
import os
import cv2
from cv_bridge import CvBridge

def main(args):
    parser = argparse.ArgumentParser(
        description='Extract image (png) and pointcloud (pcd) data from sensor messages in a directory of ig2 bags.')
    parser.add_argument('-f', '--folder', help='folder to recursively search for bags', required=True)
    parser.add_argument("-i", "--image_number", type=int, help ="maximum number of images to extract per bag", default=None)
    parser.add_argument(
        '-c', '--camera_topics', nargs='+', help='whitespace separated list of image_raw topics to try to extract', default=["/F1/image_raw", "/F2/image_raw", "/F3/image_raw"])
    parser.add_argument(
        '-l', '--lidar_topics', nargs='+', help='whitespace separated list of pointcloud2 topics to try to extract', default=["/lidar_h/velodyne_points", "/lidar_v/velodyne_points"])
    args = parser.parse_args()

    folder = os.path.dirname(args.folder)

    image_number = args.image_number
    camera_topics = args.camera_topics
    lidar_topics = args.lidar_topics
    
    bridge = CvBridge()

    for root, dirs, files in os.walk(folder):
        for file in files:
            path = os.path.join(root, file)
            name, ext = os.path.splitext(path)
            if ext == ".bag":
                bag = rosbag.Bag(path)
                n = 1
                for topic, msg, t in bag.read_messages(camera_topics):
                    if image_number is not None and n > image_number:
                        break
                    cv_img = bridge.imgmsg_to_cv2(msg)
                    img_path = os.path.join(os.path.dirname(path), str(n) + ".png")                   
                    cv2.imwrite(img_path, cv_img)
                    n += 1

if __name__ == "__main__":
    main(sys.argv[1:])
