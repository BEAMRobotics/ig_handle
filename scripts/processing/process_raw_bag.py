import rosbag
import sys
import argparse
import os
import warnings


def topic_to_dict(bag: rosbag.Bag, topics, type="msg") -> dict:
    """
    Return dictionary containing topics

    Args:
        bag (rosbag.Bag): rosbag
        topics (array): topics
        type (string: type of information to store
          "t" stored serialization time
          "msg" stores messages

    Returns:
        dictionary (dict): keys are monotonically increasing, values are topic arrays
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
                    f"{type} must be either ""msg"" or ""t""", Warning)
                continue

        dictionary[i] = array
    return dictionary


def restamp(bag, outbag, data_topics, time_topics):
    # ensure each data topic has a time topics
    if (len(data_topics) != len(time_topics)):
        raise Exception(
            "Length of time and data topic arguments is not the same.")

    # convert time and data topics to dictionaries
    stamps_msg = topic_to_dict(bag, time_topics)
    stamps_t = topic_to_dict(bag, time_topics, "t")
    messages_msg = topic_to_dict(bag, data_topics)
    messages_t = topic_to_dict(bag, data_topics, "t")

    # clip data messages that are serialized before stamps due to teensy startup
    for i, topic in enumerate(data_topics):
        if len(messages_msg[i]) == 0:
            warnings.warn(
                f"{topic} contains no messages and will not be restamped", Warning)
            continue

        while messages_t[i][0] < stamps_t[i][0]:
            try:
                messages_msg[i].pop(0)
                messages_t[i].pop(0)
            except:
                warnings.warn(
                    f"All messages on {topic} are recorded before their associated time topic and will not be restamped", Warning)

    # associate stamps with messages assuming first-in-first-out queue (i.e no signal dropout)
    for i, topic in enumerate(data_topics):
        for j in range(len(messages_msg[i])):
            try:
                time_msg = stamps_msg[i].pop(0)
                data_msg = messages_msg[i].pop(0)
            except:
                warnings.warn("Can't stamp " + topic + " seq " +
                              str(data_msg.header.seq) + ": ran out of timestamps.", Warning)
                continue

            data_msg.header.stamp = time_msg.time_ref
            outbag.write(topic, data_msg, time_msg.time_ref)

    # write remaining topics
    for topic, msg, t in bag.read_messages():
        if topic not in data_topics and topic not in time_topics:
            outbag.write(topic, msg, t)

    return outbag


def main(args):
    parser = argparse.ArgumentParser(
        description='This script is used to post-process a raw bag from ig2. Currently, this only restamps topics with their appropriate reference times.')

    parser.add_argument('-b', '--bag', help='input bag file', required=True)
    parser.add_argument(
        '-d', '--data_topics', nargs='+', help='whitespace separated list of sensor message topics', default=["/imu/data", "/F1/image_raw/compressed", "/F2/image_raw/compressed", "/F3/image_raw/compressed", "/F4/image_raw/compressed"])
    parser.add_argument(
        '-t', '--time_topics', nargs='+', help='whitespace separated list of time reference topics', default=["/imu/imu_time", "/cam/cam_time", "/cam/cam_time", "/cam/cam_time", "/cam/cam_time"])

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
