import rosbag
import sys
import argparse
import os

from utils import *

def restamp(bag, outbag, data_topics, time_topics):
    if (len(data_topics) != len(time_topics)):
        raise Exception("Length of time and data topic arguments is not the same.")

    stamps = dict()
    for i, topic in enumerate(time_topics):
        stamps[i] = topic_to_array(bag, topic)

    for topic, msg, t in bag.read_messages():
        if topic in data_topics:
            try:
                time_msg = stamps[data_topics.index(topic)].pop(0)
            except:
                print("Can't stamp " + topic + " seq " + str(msg.header.seq) + ": ran out of timestamps.")
                continue
        
            msg.header.stamp = time_msg.time_ref
            outbag.write(topic, msg, time_msg.time_ref)

        # preserve unspecified topics
        elif topic not in time_topics:
            outbag.write(topic, msg, msg.header.stamp)

    return outbag
    
        
def main(args):
    parser = argparse.ArgumentParser(
        description='Restamp and reserialize data topics with corresponding TimeReference topics.  Unspecified topics are only reserialized.')
    parser.add_argument('-b', '--bag', help='input bag file', required=True)
    parser.add_argument(
        '-d', '--data_topics', nargs='+', help='whitespace separated list of topics', required=True)
    parser.add_argument(
        '-t', '--time_topics', nargs='+', help='whitespace separated list of topics', required=True)

    args = parser.parse_args()

    data_topics = args.data_topics
    time_topics = args.time_topics
  
    bag = rosbag.Bag(args.bag)
    folder = os.path.dirname(args.bag)
    outfile = os.path.join(folder, "output.bag")
    outbag = rosbag.Bag(outfile, "w")

    outbag = restamp(bag, outbag, data_topics, time_topics)
    outbag.close()

if __name__ == "__main__":
    main(sys.argv[1:])
