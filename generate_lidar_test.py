import numpy as np
import math 
f = open('test_log.log', 'w', encoding="utf-8")
x =0
y = 0
theta = 0
v = 0.5
dt = 0.1
s = 0.1

for t in np.arange(0.0, 10.0, dt):
  x += ((math.cos(theta))*v)*dt
  y += ((math.sin(theta))*v)*dt
  theta += ((math.tan(s))/0.32)*dt
  f.write(str(t)+" "+str(x)+" "+str(y)+" "+str(v)+" "+str(s))
  for i in np.arange(0.0, ((math.pi)*2.0)/4, 1.0):
    d = -1
    
    for j in np.arange(0.0, 12.0, 0.25):
      if False:
        d = j
        break
    f.write(" "+d)
f.write(d+ "\n")

  
  
  