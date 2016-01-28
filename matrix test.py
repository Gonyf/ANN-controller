import numpy as np

x = np.matrix( ((1,2,3,4,5)) )
y = np.matrix( ((1,2,3,4,5)) )
w 	  = np.matrix([i+1 for i in range(5)]) #pole, cart, pole velocity, cart velocity
t	  = np.matrix([i+1 for i in range(5)])
print w * np.transpose(t)
print w[0,2]