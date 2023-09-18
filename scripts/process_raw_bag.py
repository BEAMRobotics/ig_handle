import rospy
import rosbag
import sys
import argparse
import os
import warnings

import numpy as np


def topic_to_dict(bag: rosbag.Bag, topics: list, type="msg") -> dict:
    """
    Return dictionary containing topics

    Args:
        bag (rosbag.Bag): rosbag
        topics (list): topics
        type (string): type of information to store
          "t" stores serialization time
          "msg" stores messages

    Returns:
        dictionary (dict): keys are monotonically increasing, values are topic lists
    """
    dictionary = dict()
    for i, topic in enumerate(topics):
        array = []
        for topic, msg, t in bag.read_messages([topic]):
            if type == "msg":
                array.append(msg)
            elif type == "t":
                array.append(t)
            else:
                warnings.warn(
                    f"{type} must be either ""msg"" or ""t"".", Warning)
                continue
        dictionary[i] = array

    return dictionary


class IgPostProcess:
    def __init__(self, args):
        # member variables
        self.bag = rosbag.Bag(args.bag)
        self.data_restamp_topics = args.data_restamp_topics
        self.time_restamp_topics = args.time_restamp_topics
        self.data_interp_topics = args.data_interp_topics
        self.time_interp_topics = args.time_interp_topics

        # ensure each data topic has a time topic
        if (len(self.data_restamp_topics) != len(self.time_restamp_topics)):
            raise Exception(
                "Length of time and data topic arguments for restamping is not the same.")

        if (len(self.data_interp_topics) != len(self.time_interp_topics)):
            raise Exception(
                "Length of time and data topic arguments for interpolation is not the same.")

        # convert time and data topics to dictionaries
        self.stamps_restamp_msg = topic_to_dict(
            self.bag, self.time_restamp_topics)
        self.stamps_restamp_t = topic_to_dict(
            self.bag, self.time_restamp_topics, "t")
        self.messages_restamp_msg = topic_to_dict(
            self.bag, self.data_restamp_topics)
        self.messages_restamp_t = topic_to_dict(
            self.bag, self.data_restamp_topics, "t")

        self.stamps_interp_msg = topic_to_dict(
            self.bag, self.time_interp_topics)
        self.messages_interp_msg = topic_to_dict(
            self.bag, self.data_interp_topics)
        self.messages_interp_t = topic_to_dict(
            self.bag, self.data_interp_topics, "t")

        # create output bag
        out_folder = os.path.dirname(self.bag.filename)
        out_file = os.path.join(out_folder, "output.bag")
        self.out_bag = rosbag.Bag(out_file, "w")

        # write remaining topics
        for topic, msg, t in self.bag.read_messages():
            is_restamp_topic = topic not in self.data_restamp_topics and topic not in self.time_restamp_topics
            is_interp_topic = topic not in self.data_interp_topics and topic not in self.time_interp_topics
            if is_restamp_topic and is_interp_topic:
                self.out_bag.write(topic, msg, t)

        # process
        self.restamp()
        self.interpolate()
        self.save()

    def restamp(self):
        """
        restamp data_restamp_topics via time_restamp_topics
        """

        # clip data messages that are serialized before stamps due to teensy startup
        for i, topic in enumerate(self.data_restamp_topics):
            if len(self.messages_restamp_msg[i]) == 0:
                warnings.warn(
                    f"{topic} contains no messages and will not be restamped.", Warning)
                continue

            if len(self.stamps_restamp_msg[i]) == 0:
                warnings.error(
                    f"{topic} contains no stamps. Data collection is corrupt.", Warning)
                continue

            while self.messages_restamp_t[i][0] < self.stamps_restamp_t[i][0]:
                try:
                    self.messages_restamp_msg[i].pop(0)
                    self.messages_restamp_t[i].pop(0)
                except:
                    warnings.warn(
                        f"All messages on {topic} are recorded before their associated time topic and will not be restamped.", Warning)

            num_stamps = len(self.stamps_restamp_msg[i])
            num_messages = len(self.messages_restamp_msg[i])
            if (abs(num_stamps - num_messages) > 1):
                warnings.warn(
                    f"{topic} contains {num_stamps} stamps and {num_messages} messages. There was signal dropout during data collection and the data is corrupt.", Warning)
                continue

        # associate stamps with messages assuming first-in-first-out queue (i.e no signal dropout)
        for i, topic in enumerate(self.data_restamp_topics):
            while len(self.messages_restamp_msg[i]) > 0:
                try:
                    time_msg = self.stamps_restamp_msg[i].pop(0)
                    data_msg = self.messages_restamp_msg[i].pop(0)
                except:
                    warnings.warn("Can't stamp " + topic + " seq " +
                                  str(data_msg.header.seq) + ": ran out of timestamps.", Warning)
                    continue

                serialization_stamp = data_msg.header.stamp
                data_msg.header.stamp = time_msg.time_ref
                self.out_bag.write(topic, data_msg, serialization_stamp)

    def interpolate(self):
        """
        interpolate data_interp_topics via time_interp_topics
        """
        for i, topic in enumerate(self.data_interp_topics):
            # instantiate lists
            t1_nsec_ros = []
            t_nsec_ros = []
            t_nsec_pps = []

            # get interest points x1
            for j in range(len(self.messages_interp_msg[i])):
                t1_nsec_ros.append(
                    (self.messages_interp_msg[i][j].header.stamp.to_sec()))

            # get interpolation (x,y) points
            for j in range(len(self.stamps_interp_msg[i])):
                t_nsec_ros.append(
                    (self.stamps_interp_msg[i][j].header.stamp.to_sec()))
                t_nsec_pps.append(
                    (self.stamps_interp_msg[i][j].time_ref.to_sec()))

            # interpolate
            t1_nsec_pps = np.interp(
                t1_nsec_ros, t_nsec_ros, t_nsec_pps, left=np.nan, right=np.nan)

            # restamp data_interp_topics using interpolated time
            for j in range(len(self.messages_interp_msg[i])):
                # copy
                data_msg = self.messages_interp_msg[i][j]
                t_msg = self.messages_interp_t[i][j]

                # restamp
                if not np.isnan(t1_nsec_pps[j]):
                    data_msg.header.stamp = rospy.Time.from_sec(t1_nsec_pps[j])
                    self.out_bag.write(topic, data_msg, t_msg)

    def save(self):
        """
        save out_bag to disk
        """
        self.out_bag.close()


def main(args):
    parser = argparse.ArgumentParser(
        description='This script is used to post-process a raw bag from ig_handle. This script restamps topics with their appropriate reference times and interpolates reference times for soft-synchronized sensors.')
    parser.add_argument('-b', '--bag', help='input bag file', required=True)
    parser.add_argument(
        '-dr', '--data_restamp_topics', nargs='+', help='list of sensor message topics to be restamped', default=["/imu/data", "/F1/image_raw/compressed", "/F2/image_raw/compressed", "/F3/image_raw/compressed", "/F4/image_raw/compressed"])
    parser.add_argument(
        '-tr', '--time_restamp_topics', nargs='+', help='list of time reference topics corresponding to restamped sensor message topics', default=["/imu/time", "/cam/time", "/cam/time", "/cam/time", "/cam/time"])
    parser.add_argument(
        '-di', '--data_interp_topics', nargs='+', help='list of sensor message topics to be interpolated', default=["/DT100/sonar_scans"])
    parser.add_argument(
        '-ti', '--time_interp_topics', nargs='+', help='list of time reference topics corresponding to interpolated sensor message topics', default=["/pps/time"])

    # parse args
    args = parser.parse_args()

    # process
    obj = IgPostProcess(args)


if __name__ == "__main__":
    main(sys.argv[1:])
