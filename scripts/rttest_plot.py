import argparse
import matplotlib.pyplot as pyplot
import numpy

def main():
    parser = argparse.ArgumentParser(description="Plot rttest output")
    parser.add_argument('filename', metavar='filename')
    args = parser.parse_args();
    filename = args.filename
    rawlines = []

    with open(filename) as f:
        rawlines = f.readlines()
    rawlines = [line.rstrip().split(" ") for line in rawlines]
    array = numpy.array(rawlines)
    # Units for time and latency are nanoseconds
    time = array[1:, 1].astype(long)
    latency = numpy.absolute(array[1:, 2].astype(long))
    # Plot abs( column 2 (latency, ns) ) against column 1 (time, ns)
    pyplot.figure(1)
    pyplot.plot(time, latency)
    pyplot.title('Scheduling latency vs. time for: ' + filename)
    pyplot.xlabel('Time (ns)')
    pyplot.ylabel('Latency (ns)')
    pyplot.show()

if __name__ == "__main__":
    main()
