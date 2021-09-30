import rosbag
import sys
import argparse
import os

from utils import *

def stamp_from_serialization(bag, outbag, topics):
    for topic, msg, t in bag.read_messages():
        if topic in topics:
            msg.header.stamp = t
            # preserve unspecified topics
            outbag.write(topic, msg, t)

        else:
            outbag.write(topic, msg, t)

    return outbag
    
        
def main(args):
    parser = argparse.ArgumentParser(
        description='Restamp given topics with serialization time to repair synchronization bugs.  Unspecified topics are preserved.')
    parser.add_argument('-b', '--bag', help='input bag file', required=True)
    parser.add_argument(
        '-t', '--topics', nargs='+', help='whitespace separated list of topics', required=True)

    args = parser.parse_args()

    topics = args.topics
  
    bag = rosbag.Bag(args.bag)
    folder = os.path.dirname(args.bag)
    outfile = os.path.join(folder, "output.bag")
    outbag = rosbag.Bag(outfile, "w")

    outbag = stamp_from_serialization(bag, outbag, topics)
    outbag.close()

if __name__ == "__main__":
    main(sys.argv[1:])