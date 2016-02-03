import numpy as np

x = np.matrix( ((1,2,3,4,5)) )
y = np.matrix( ((1,2,3,4,5)) )
w 	  = np.matrix([i+1 for i in range(5)]) #pole, cart, pole velocity, cart velocity
t	  = np.matrix([i+1 for i in range(5)])
wg 	  = np.matrix([1.0 for i in range(4)])
we 	  = np.matrix([0.1*i for i in range(4)]) #pole, cart, pole velocity, cart velocity
we = np.r_[we,we]
print we
z = we * np.transpose(wg)
print z
print np.tanh(z)
print np.sign(-10)
print np.sign(12)
