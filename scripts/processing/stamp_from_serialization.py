import rosbag
import rospy
import sys
import argparse
import os

# delay (in sec) in sending NMEA message as per /main/main.ino
nmea_delay = rospy.Duration.from_sec(0.055)


def stamp_from_serialization(bag, outbag, topics):
    for topic, msg, t in bag.read_messages():
        if topic in topics:
            msg.header.stamp = t - nmea_delay
            outbag.write(topic, msg, t - nmea_delay)
        else:
            # preserve unspecified topics
            outbag.write(topic, msg, t)

    return outbag


def main(args):
    parser = argparse.ArgumentParser(
        description='Restamp LiDAR topics with serialization time adjusted for NMEA delay. All other topics are preserved.')
    parser.add_argument('-b', '--bag', help='input bag file', required=True)
    parser.add_argument(
        '-t', '--topics', nargs='+', help='whitespace separated list of LiDar topics', default=["/lidar_h/velodyne_packets", "/lidar_h/velodyne_points", "/lidar_v/velodyne_packets", "/lidar_v/velodyne_points"])

    args = parser.parse_args()

    bag = rosbag.Bag(args.bag)
    folder = os.path.dirname(args.bag)
    outfile = os.path.join(folder, "output_softsynch.bag")
    outbag = rosbag.Bag(outfile, "w")

    topics = args.topics

    # restamp LiDAR messages using serialization time
    outbag = stamp_from_serialization(bag, outbag, topics)
    outbag.close()


if __name__ == "__main__":
    main(sys.argv[1:])
