#!/usr/bin/python2

import yaml
import rosbag
import argparse
import re

parser = argparse.ArgumentParser(description="Greps /rosout for a specific message and reports its timestamps")
parser.add_argument("regex", help="regex pattern to search for")
parser.add_argument("bagfiles", nargs='+', help="input bag file")
args = parser.parse_args()

bagfiles = args.bagfiles
regex = args.regex

for bagfile in bagfiles:
    print("From " + str(bagfile) + ":")
    info_dict = yaml.load(rosbag.Bag(bagfile, 'r')._get_yaml_info())
    for topic, msg, t in rosbag.Bag(bagfile).read_messages():
        if topic == "/rosout":
            if re.search(regex, msg.msg) or re.search(regex, msg.name):
                seconds_from_start = t.to_sec() - info_dict["start"]
                print("[" + str(seconds_from_start) + "] " + msg.msg)
