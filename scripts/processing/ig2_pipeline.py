import rosbag
import sys
import argparse
import os

from restamp import restamp


def main(args):
    parser = argparse.ArgumentParser(
        description='Pipeline of post processing steps for raw ig2 bags.')
    parser.add_argument('-b', '--bag', help='input bag file', required=True)
    parser.add_argument(
        '-d', '--data_topics', nargs='+', help='whitespace separated list of sensor message topics', default=["/imu/data", "/F1/image_raw", "/F2/image_raw", "/F3/image_raw"])
    parser.add_argument(
        '-t', '--time_topics', nargs='+', help='whitespace separated list of time reference topics', default=["/imu/imu_time", "/F1/cam_time", "/F2/cam_time", "/F3/cam_time"])

    args = parser.parse_args()

    bag = rosbag.Bag(args.bag)
    folder = os.path.dirname(args.bag)
    outfile = os.path.join(folder, "output.bag")
    outbag = rosbag.Bag(outfile, "w")

    time_topics = args.time_topics
    data_topics = args.data_topics
    
    # pair and restamp data/time message couples
    outbag = restamp(bag, outbag, data_topics, time_topics)
    
    outbag.close()


if __name__ == "__main__":
    main(sys.argv[1:])
