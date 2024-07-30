import numpy as np
import csv

log = open('test_log.log', 'w')
writer = csv.writer(log)
for t in np.arange(0.0, 6.0, 0.1):
  writer.writerow((str(t) + ' 0.0 0.0 0.5 0.1 0.0 1.0 0.0 0.0\n').split())