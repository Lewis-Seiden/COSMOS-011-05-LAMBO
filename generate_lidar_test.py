import numpy as np
import math 
import random
import csv
f = open('test_log.log', 'w', encoding="utf-8")
log = csv.writer(f)
x = 0.0
y = 0.0
theta = 0.0
v = 0.75
dt = 0.2
s = 0.05

obs = [(1.0,0.0,2.0), (1.0, 0.0, -2.0)]#, (2.0, 12.0, 8.0)]

for t in np.arange(0.0, 10.0, dt):
  res = []
  s = (random.random() - 0.2) * 0.4
  x += ((math.cos(theta))*v)*dt
  y += ((math.sin(theta))*v)*dt
  theta += ((math.tan(s))/0.32)*dt*v
  res.extend([str(t),str(x),str(y),str(v),str(s)])
  for phi in np.linspace(0.0, ((math.pi)*2.0), 64, endpoint=False):
    d = 0

    for j in np.linspace(0.0, 12.0, 100):
      hasCollided = False
      for k in obs:
        cx = math.cos((theta + phi))* j
        cy = -math.sin((theta + phi))* j
        isColliding = ((k[2]-(y+cy))**2)+((k[1]-(x+cx))**2) < k[0]**2

        hasCollided |= isColliding
      if hasCollided:
        if j != 0.0:
          d = j
        break
    res.append(str(d * math.cos(phi)))
    res.append(str(d * math.sin(phi)))
  log.writerow(res)

  
  
  