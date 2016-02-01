import numpy as np

x = np.matrix( ((1,2,3,4,5)) )
y = np.matrix( ((1,2,3,4,5)) )
w 	  = np.matrix([i+1 for i in range(5)]) #pole, cart, pole velocity, cart velocity
t	  = np.matrix([i+1 for i in range(5)])
wg 	  = np.matrix([1.0 for i in range(4)])
we 	  = np.matrix([2.0 for i in range(4)]) #pole, cart, pole velocity, cart velocity
we = np.r_[we,we]
print we * np.transpose(wg)
print w[0,2]