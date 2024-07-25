import numpy as np

log = open('test_log.log', 'w')
for t in np.arange(0.0, 6.0, 0.1):
  log.write(str(t) + ' 0.0 0.0 0.5 0.1 0.0 1.0 0.0 0.0\n')