import rospy
import rosbag
import sys
import argparse
import os
import numpy as np
import logging
logging.basicConfig(level=logging.INFO)


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
                logging.warning(f"{type} must be either ""msg"" or ""t"".")
                continue
        dictionary[i] = array

    return dictionary


class IgPostProcess:
    def __init__(self, args):
        # get args
        self.bag = rosbag.Bag(args.bag)
        self.bag_start = args.bag_start
        self.bag_end = args.bag_end
        self.data_restamp_topics = args.data_restamp_topics
        self.time_restamp_topics = args.time_restamp_topics
        self.clip_restamp_topics = args.clip_restamp_topics
        self.data_interp_topics = args.data_interp_topics
        self.time_interp_topics = args.time_interp_topics

        logging.info("Parsing arguments...")

        # set bag start and end time
        if self.bag_start < 0:
            self.bag_start_time = rospy.Time(self.bag.get_start_time())
        else:
            self.bag_start_time = rospy.Time(
                self.bag.get_start_time()) + rospy.Duration.from_sec(self.bag_start)

        if self.bag_end < 0:
            self.bag_end_time = rospy.Time(self.bag.get_end_time())
        else:
            self.bag_end_time = rospy.Time(
                self.bag.get_start_time()) + rospy.Duration.from_sec(self.bag_end)

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
        os.system(f"rm -rf {out_file}")
        self.out_bag = rosbag.Bag(out_file, "w")

        # write remaining topics
        for topic, msg, t in self.bag.read_messages():
            is_not_restamp_topic = topic not in self.data_restamp_topics and topic not in self.time_restamp_topics
            is_not_interp_topic = topic not in self.data_interp_topics and topic not in self.time_interp_topics
            if is_not_restamp_topic and is_not_interp_topic and t >= self.bag_start_time and t <= self.bag_end_time:
                self.out_bag.write(topic, msg, msg.header.stamp)

        logging.info(f"Parsing complete. Processing {self.bag.filename}...")

        # process
        self.restamp()
        self.interpolate()
        self.save()

    def restamp(self):
        """
        restamp data_restamp_topics via time_restamp_topics
        """

        # main restamp loop
        for i, topic in enumerate(self.data_restamp_topics):
            logging.info(f"Checking {topic} for restamp...")

            # check data messages
            if len(self.messages_restamp_msg[i]) == 0:
                logging.warning(
                    f"{topic} contains no messages and will not be restamped.")
                continue

            # check stamps messages
            if len(self.stamps_restamp_msg[i]) == 0:
                logging.error(
                    f"{topic} contains no stamps and will not be restamped. Data collection is corrupt.")
                continue

            # clip data messages that are serialized before stamps due to teensy startup
            while self.messages_restamp_t[i][0] < self.stamps_restamp_t[i][0]:
                try:
                    self.messages_restamp_msg[i].pop(0)
                    self.messages_restamp_t[i].pop(0)
                except:
                    logging.warning(
                        f"All messages on {topic} are recorded before their associated time topic and will not be restamped.")
                    continue

            # clip data and time messages before a use-defined clip time
            bag_clip_time = rospy.Time(
                self.bag.get_start_time()) + rospy.Duration.from_sec(self.clip_restamp_topics[i])

            while self.messages_restamp_t[i][0] < bag_clip_time:
                try:
                    self.messages_restamp_msg[i].pop(0)
                    self.messages_restamp_t[i].pop(0)
                except:
                    logging.warning(
                        f"All messages on {topic} are recorded before the user specified clip time of {bag_clip_time}.")
                    continue
            while self.stamps_restamp_t[i][0] < bag_clip_time:
                try:
                    self.stamps_restamp_msg[i].pop(0)
                    self.stamps_restamp_t[i].pop(0)
                except:
                    logging.warning(
                        f"All messages on {topic} are recorded before the user specified clip time of {bag_clip_time}.")
                    continue

            # clip data and stamp messages after bag end time
            while self.messages_restamp_t[i][-1] > self.bag_end_time:
                try:
                    self.messages_restamp_msg[i].pop(-1)
                    self.messages_restamp_t[i].pop(-1)
                except:
                    logging.warning(
                        f"No messages on {topic} remain when processing bag for {self.bag_duration} seconds from start and will not be restamped.")
                    continue
            while self.stamps_restamp_t[i][-1] > self.bag_end_time:
                try:
                    self.stamps_restamp_msg[i].pop(-1)
                    self.stamps_restamp_t[i].pop(-1)
                except:
                    logging.warning(
                        f"No messages on {topic} remain when processing bag for {self.bag_duration} seconds from start and will not be restamped.")
                    continue

            # check for signal dropout
            num_stamps = len(self.stamps_restamp_msg[i])
            num_messages = len(self.messages_restamp_msg[i])
            if (abs(num_stamps - num_messages) > 1):
                logging.warning(
                    f"{topic} contains {num_stamps} stamps and {num_messages} messages. There was signal dropout during data collection and {topic} will not be restamped. ")
                continue

            # associate stamps with messages assuming FIFO queue
            logging.info("Check completed, restamping...")
            while len(self.messages_restamp_msg[i]) > 0:
                try:
                    time_t = self.stamps_restamp_t[i].pop(0)
                    data_t = self.messages_restamp_t[i].pop(0)
                    time_msg = self.stamps_restamp_msg[i].pop(0)
                    data_msg = self.messages_restamp_msg[i].pop(0)
                except:
                    logging.warning(
                        f"Restamping stopped at {topic}:{data_msg.header.seq}. Ran out of stamps.")
                    break
                data_msg.header.stamp = time_msg.time_ref
                if (min(time_t, data_t) < self.bag_start_time):
                    continue
                self.out_bag.write(topic, data_msg, time_msg.time_ref)
            logging.info("Restamping completed.")

    def interpolate(self):
        """
        interpolate data_interp_topics via time_interp_topics
        """

        # main interpolation loop
        for i, topic in enumerate(self.data_interp_topics):
            logging.info(f"Checking {topic} for interpolation...")

            # check data messages
            if len(self.messages_interp_msg[i]) == 0:
                logging.warning(
                    f"{topic} contains no messages and will not be interpolated.")
                continue

            # check stamps messages
            if len(self.stamps_interp_msg[i]) < 2:
                logging.error(
                    f"{topic} contains fewer than two stamps required for interpolation.")
                continue

            logging.info("Check completed, interpolating...")

            # instantiate lists
            t1_nsec_ros = []  # x1
            t_nsec_ros = []  # x
            t_nsec_pps = []  # y

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

            # interpolate points y1
            t1_nsec_pps = np.interp(
                t1_nsec_ros, t_nsec_ros, t_nsec_pps, left=np.nan, right=np.nan)

            # restamp data_interp_topics using interpolated time
            for j in range(len(self.messages_interp_msg[i])):
                if not np.isnan(t1_nsec_pps[j]):
                    time_msg = rospy.Time.from_sec(t1_nsec_pps[j])
                    if time_msg >= self.bag_start_time and time_msg <= self.bag_end_time:
                        data_msg = self.messages_interp_msg[i][j]
                        data_msg.header.stamp = time_msg
                        self.out_bag.write(topic, data_msg, time_msg)
            logging.info("Interpolation completed.")

    def save(self):
        """
        save out_bag to disk
        """
        self.out_bag.close()
        logging.info(f"{self.out_bag.filename} saved to disk.")


def main(args):
    parser = argparse.ArgumentParser(
        description='This script is used to post-process a raw bag from ig_handle. This script restamps topics with their appropriate reference times and interpolates reference times for soft-synchronized sensors.')
    parser.add_argument('--bag', type=str, required=True,
                        help='input bag file')
    parser.add_argument(
        '--data_restamp_topics', default=["/imu/data", "/F1/image_raw/compressed", "/F2/image_raw/compressed", "/F3/image_raw/compressed", "/F4/image_raw/compressed"], type=str, nargs='+', help='list of sensor data message topics to be restamped')
    parser.add_argument(
        '--time_restamp_topics', default=["/imu/time", "/cam/time", "/cam/time", "/cam/time", "/cam/time"], type=str, nargs='+', help='list of time reference topics corresponding to restamped sensor data message topics')
    parser.add_argument(
        '--data_interp_topics', default=["/DT100/sonar_scans"], type=str, nargs='+', help='list of sensor data message topics to be interpolated')
    parser.add_argument(
        '--time_interp_topics', default=["/pps/time"], type=str, nargs='+', help='list of time reference topics corresponding to interpolated sensor data message topics')
    parser.add_argument(
        '--clip_restamp_topics', default=[0.0, 0.0, 0.0, 0.0, 0.0], type=float, nargs='+', help='list of times (in seconds) for sensor data message topics to be clipped')
    parser.add_argument(
        '--bag_start', default=-1.0, type=float, help='start time (in seconds) for output.bag according to serialization time. Default is start of bag.')
    parser.add_argument(
        '--bag_end', default=-1.0, type=float, help='end time (in seconds) for output.bag according to serialization time. Default is end of bag.')

    # parse args and process
    args = parser.parse_args()
    obj = IgPostProcess(args)


if __name__ == "__main__":
    main(sys.argv[1:])
