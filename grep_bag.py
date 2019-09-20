#!/usr/bin/python2

import yaml
import rosbag
import argparse
import re

parser = argparse.ArgumentParser(description="Greps /rosout for a specific message and reports its timestamps")
parser.add_argument("bagfile", nargs=1, help="input bag file")
parser.add_argument("regex", nargs=1, help="regex pattern to search for")
args = parser.parse_args()

bagfile = args.bagfile[0]
regex = args.regex[0]

info_dict = yaml.load(rosbag.Bag(bagfile, 'r')._get_yaml_info())

# import pdb

for topic, msg, t in rosbag.Bag(bagfile).read_messages():
    if topic == "rosout":
        if re.search(regex, msg.msg) or re.search(regex, msg.name):
            seconds_from_start = t.to_sec() - info_dict["start"]
            print("[" + str(seconds_from_start) + "] " + msg.msg)
            # pdb.set_trace()
