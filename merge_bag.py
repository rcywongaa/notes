#!/usr/bin/env python
# Modified from https://github.com/udacity/self-driving-car/blob/master/image-localization/community-code/roboauto/scripts/bagmerge.py
import sys
import roslib;
import rospy
import rosbag
from rospy import rostime
import argparse
import os

def parse_args():
    parser = argparse.ArgumentParser(
        prog = 'bagmerge.py',
        description='Merges two or more bagfiles.')
    parser.add_argument('-o', type=str, help='name of the output file', 
        default = "merged.bag", metavar = "output_file")
    parser.add_argument('bagfiles', type=str, nargs='+', help='path to two or more bagfiles')
    args = parser.parse_args()
    return args

def get_next(bag_iter, reindex = False, 
        main_start_time = None, start_time = None, 
        topics = None):
    try:
        result = bag_iter.next()
        if topics != None:
            while not result[0] in topics:
                result = bag_iter.next()
        if reindex:
            return (result[0], result[1], 
                result[2] - start_time + main_start_time)
        return result
    except StopIteration:
        return None

def merge_bag(main_bagfile, bagfile, outfile = None, topics = None, 
        reindex = True):
    #get min and max time in bagfile
    main_limits = get_limits(main_bagfile)
    limits = get_limits(bagfile)
    #check output file
    if outfile == None:
        pattern = main_bagfile + "_merged_%i.bag"
        outfile = main_bagfile + "_merged.bag"
        index = 0
        while (os.path.exists(outfile)):
            outfile = pattern%index
            index += 1
    #output some information
    print "merge bag %s in %s"%(bagfile, main_bagfile)
    print "topics filter: ", topics
    print "writing to %s."%outfile
    #merge bagfile
    outbag = rosbag.Bag(outfile, 'w')
    main_bag = rosbag.Bag(main_bagfile).__iter__()
    bag = rosbag.Bag(bagfile).__iter__()
    main_next = get_next(main_bag)
    next = get_next(bag, reindex, main_limits[0], limits[0], topics)
    try:
        while main_next != None or next != None:
            if main_next == None:
                outbag.write(next[0], next[1], next[2])
                next = get_next(bag, reindex, main_limits[0], limits[0], topics)
            elif next == None:
                outbag.write(main_next[0], main_next[1], main_next[2])
                main_next = get_next(main_bag)
            elif next[2] < main_next[2]:
                outbag.write(next[0], next[1], next[2])
                next = get_next(bag, reindex, main_limits[0], limits[0], topics)
            else:
                outbag.write(main_next[0], main_next[1], main_next[2])
                main_next = get_next(main_bag)
    finally:
        outbag.close()

def get_limits(bagfile):
    print "Determine start and end index of %s..."%bagfile
    end_time = None
    start_time = None

    for topic, msg, t in rosbag.Bag(bagfile).read_messages():
        if start_time == None or t < start_time:
            start_time = t
        if end_time == None or t > end_time:
            end_time = t
    return (start_time, end_time)
    
if __name__ == "__main__":
    args = parse_args()
    current_intermediate = "intermediate0"
    # merge_bag(args.bagfiles[0], args.bagfiles[1], outfile = current_intermediate, reindex = True)
    # for i in range(1, len(args.bagfiles)):
        # new_intermediate = "intermediate" + str(i)
        # merge_bag(current_intermediate, args.bagfiles[i], outfile = new_intermediate, reindex = True)
        # os.remove(current_intermediate)
        # current_intermediate = new_intermediate
    # os.rename(current_intermediate, "merged.bag")


    input_file = args.bagfiles[0]
    for i in range(1, len(args.bagfiles)):
        if i == len(args.bagfiles) - 1:
            reindex = True
        else:
            reindex = False

        output_file = "intermediate" + str(i)
        merge_bag(input_file, args.bagfiles[i], outfile = output_file, reindex = reindex)

        if i > 1:
            os.remove(input_file)

        input_file = output_file

    os.rename(output_file, "merged.bag")
