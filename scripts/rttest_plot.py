import argparse
import matplotlib.pyplot as pyplot
import numpy

def main():
    parser = argparse.ArgumentParser(description="Plot rttest output")
    parser.add_argument('filename', metavar='filename')
    parser.add_argument('-s', '--show', help="Show plots", action="store_true")
    parser.add_argument('-o', '--outfile', help="Name of file to write plot output", default=None)
    args = parser.parse_args();
    filename = args.filename
    show = args.show
    outfile = filename + "_plot"
    if args.outfile is not None:
        outfile = args.outfile

    rawlines = []

    with open(filename) as f:
        rawlines = f.readlines()
    rawlines = [line.rstrip().split(" ") for line in rawlines]
    array = numpy.array(rawlines)
    # Units for time and latency are nanoseconds
    time = array[1:, 1].astype(long)
    latency = numpy.absolute(array[1:, 2].astype(long))
    min_pagefaults = array[1:, 3].astype(long)
    maj_pagefaults = array[1:, 4].astype(long)
    # Plot abs( column 2 (latency, ns) ) against column 1 (time, ns)
    pyplot.figure(1)
    pyplot.plot(time, latency)
    pyplot.title('Scheduling latency vs. time for: ' + filename)
    pyplot.xlabel('Time (ns)')
    pyplot.ylabel('Latency (ns)')
    if show:
      pyplot.show()
    pyplot.savefig(outfile + "_latency.svg")
    # Plot column 3 (minor pagefaults) against column 1 (time, ns)
    pyplot.figure(2)
    pyplot.plot(time, min_pagefaults)
    pyplot.title('Minor pagefaults vs. time for: ' + filename)
    pyplot.xlabel('Time (ns)')
    pyplot.ylabel('Minor pagefaults')
    pyplot.savefig(outfile + "_minflt.svg")

    if show:
      pyplot.show()
    # Plot column 3 (major pagefaults) against column 1 (time, ns)
    pyplot.figure(3)
    pyplot.plot(time, maj_pagefaults)
    pyplot.title('Major pagefaults vs. time for: ' + filename)
    pyplot.xlabel('Time (ns)')
    pyplot.ylabel('Major pagefaults')
    if show:
      pyplot.show()
    pyplot.savefig(outfile + "_majflt.svg")


if __name__ == "__main__":
    main()
