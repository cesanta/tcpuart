#!/usr/bin/python

# Adds time delta (ms) to Wireshark CSV exports

import csv
import sys

writer = csv.writer(sys.stdout)

with open(sys.argv[1], 'r') as f:
  reader = csv.reader(f)
  reader.next()  # Skip header
  pt = 0.0
  for row in reader:
      t = float(row[1])
      td_ms = (t - pt) * 1000
      #writer.writerow(row[:2] + ["%.03f" % (td * 1000)] + row[2:])
      writer.writerow([row[0], "%.03f" % td_ms])
      if td_ms > 100:
          print >>sys.stderr, row
      pt = t
