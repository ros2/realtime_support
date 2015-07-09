#!/usr/bin/env python

# measure average jitter of an executable
# get_average_jitter.py <iterations> executable <rttest args>

# Run the executable <iterations> times
# get min from "Min: <min>"
# get max from "Max: <max>"
# get mean from "Mean: <mean>"
# get stddev from "Standard deviation: <stddev>

# Keep a running tally and average over <iterations>

from __future__ import print_function
import re
import subprocess
import sys
from sys import argv

if len(argv) < 3:
    print("get_average_jitter requires at least 3 arguments.", file=sys.stderr)
    exit(1)

iterations = int(argv[1])
command = argv[2:]

keys = ["Minor pagefaults", "Major pagefaults", "Min", "Max", "Mean", "Standard deviation"]

tally = dict(zip(keys, [0.0]*len(keys)))

for i in range(iterations):
    stdoutput = subprocess.check_output(command)
    # Split on double newline
    print(stdoutput)
    outputs = stdoutput.split('\n\n')

    for output in outputs:
        for key in keys:
            match = re.search('(?<='+key+': )\w+', output)
            if match is not None:
                tally[key] += float(match.group())

# get averages
for key in keys:
    average = tally[key]/iterations
    print("Average {}: {}".format(key, str(average)))

