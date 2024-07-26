import numpy as np
import math 
f = open('test_log.log', 'w', encoding="utf-8")
x = 0.0
y = 0.0
theta = 0.0
v = 0.75
dt = 0.2
s = 0.1

obs = [(1.0,0.0,2.0), (1.0, 0.0, -2.0)]

for t in np.arange(0.0, 20.0, dt):
  x += ((math.cos(theta))*v)*dt
  y += ((math.sin(theta))*v)*dt
  theta += ((math.tan(s))/0.32)*dt*v
  f.write(str(t)+" "+str(x)+" "+str(y)+" "+str(v)+" "+str(s))
  for i in np.linspace(0.0, ((math.pi)*2.0), 64, endpoint=False):
    d = -1

    for j in np.linspace(0.0, 12.0, 100):
      hasCollided = False
      for k in obs:
        cx = math.cos((theta + i))* j
        cy = -math.sin((theta + i))* j
        isColliding = ((k[2]-(y+cy))**2)+((k[1]-(x+cx))**2) < k[0]**2

        hasCollided |= isColliding
      if hasCollided:
        if j != 0.0:
          d = j
        break
    f.write(" " + str(d))
  f.write("\n")

  
  
  