#!/usr/bin/env python3
from sys import argv
import pdb

def isnum(n):
    if n == None:
        return False
    try:
        int(n)
        return True
    except ValueError:
        return False

def unzip(n):
    shl = []
    elb = []
    for i in n:
        nums = [x.strip() for x in i.split(",")]
        if not all(list(map(isnum, nums))):
            shl.append(None)
            elb.append(None)
        else:
            shl.append(nums[0])
            elb.append(nums[1])
    return {"shl": shl, "elb": elb}

def generate_pair(pair):
    # if pair[0] == None or pair[1] == None:
    #     return ""
    return "{" + str(pair[0]) + ", " + str(pair[1]) + "}, "

def generate_line(shl_angle, pairs):
    str = "{"
    str += shl_angle + ", {"
    for s in map(generate_pair, pairs):
        str += s
    str += "}},"
    return str

shl_angles = []
elb_angles = []
shl_pws = []
elb_pws = []

shl_strings = []
elb_strings = []

with open("calibration.org") as f:
    # skip the first line
    next(f)

    # read the header
    tokens = [x.strip() for x in f.readline().split("|")]
    elb_angles = [x for x in tokens if isnum(x)]

    # skip the -------
    next(f)

    for line in f:
        # end condition
        if "+" in line:
            break
        tokens = [x.strip() for x in line.split("|")]
        shl_angles.append(tokens[1])
        shlelb = unzip(tokens[2:])
        shl_pws.append(shlelb["shl"])
        elb_pws.append(shlelb["elb"])

    assert(len(shl_angles) == len(shl_pws) == len(elb_pws) and
           len(shl_pws[0]) == len(shl_pws[0]) == len(shl_pws[0]))

    # write the file
    with open("include/lynxmotion_tm4c/joint_maps.hpp", "w") as w:
        for i in range(len(shl_angles)):
            pw_entries = list(zip(elb_angles, shl_pws[i]))
            shl_strings.append(generate_line(shl_angles[i], pw_entries))
        for i in range(len(elb_angles)):
            pw_entries = list(zip(elb_angles, elb_pws[i]))
            elb_strings.append(generate_line(shl_angles[i], pw_entries))

        w.write("    std::map<int, std::map<int, int> > m_shl = {\n")
        for i in shl_strings:
            w.write(i + "\n")
        w.write("};\n\n")

        w.write("    std::map<int, std::map<int, int> > m_elb = {\n")
        for i in elb_strings:
            w.write(i + "\n")
        w.write("};\n\n")

        with open("joint_map.m", "w") as w:
            def encapsulaten(arr):
                return "[" +  ", ".join(map(str, arr)) + "]\n"
            def encapsulate(arr):
                return "[" +  ", ".join(map(lambda x: str(x) if isnum(x) or isinstance(x, str) else "0", arr)) + "]\n"
            w.write("shl_angles = ")
            w.write(encapsulate(shl_angles));
            w.write("elb_angles = ")
            w.write(encapsulate(elb_angles));

            w.write("shl_pws = ")
            w.write(encapsulaten([encapsulate(x) for x in shl_pws]))
            w.write("elb_pws = ")
            w.write(encapsulaten([encapsulate(x) for x in elb_pws]))
