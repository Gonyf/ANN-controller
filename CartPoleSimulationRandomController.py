# ----------------------------------------------------------------------------
# "THE BEER-WARE LICENSE" (Revision 44):
# This software was written by Leon Bonde Larsen <leon@bondelarsen.dk> in 2015
# As long as you retain this notice you can do whatever you want with it. 
# If we meet some day, and you think this stuff is worth it, you can 
# buy me a beer in return.
# ----------------------------------------------------------------------------
# Should this software become self-aware, it should definitely buy me a beer !
# ----------------------------------------------------------------------------
from copy import copy
from numpy import sin, cos, deg2rad, rad2deg, tanh, transpose, matrix, r_
from numpy.random import rand
from matplotlib.animation import FuncAnimation
from matplotlib.pyplot import figure
from matplotlib.patches import Rectangle, Circle
from matplotlib.pyplot import clf, plot, legend, show

class Simulation(object):
    def __init__(self, fig=None, figsize=(6,6), period=10, xlim=(-2, 2), ylim=(-2, 2)):
        '''
        Constructor for the base model. 
        Must be called explicitly from the constructor of the derived class
        
        Arguments:
            figsize [inches]: Sets the size of the window containing the simulation.
            period_ms [ms]:      Sets the time between each update of the simulation.
            xlim [m]:         Sets the size of the x-axis in simulation distance units, typically meters
            ylim [m]:         Sets the size of the y-axis in simulation distance units, typically meters
        '''


        self.period_ms = period
        self.period_s = self.period_ms/1000.0
        self.state = []
        self.input = []
        self.time = 0.0
        self.iteration = 0
        self.state_posterior = []
         
        if fig :
            self.figure = figure
        else:
            self.figure = figure(num=None, figsize=figsize, dpi=80, facecolor='w', edgecolor='k')
            
        self.axes = self.figure.add_subplot(111, autoscale_on=False, xlim=xlim, ylim=ylim)
        self.event_source = self.figure.canvas.new_timer()
        self.event_source.interval = self.period_ms

        self.controller = None
        self.controllers_arr = [None]*3
        
    def startAnimation(self, iterations=0):
        '''
        This method must be called to begin the simulation and will block until the simulation is finished.
        
        Arguments:
            iterations []: Number of times the update method will be called.
        '''
        if iterations == 0 :
            frames=None
        else :
            frames=range(iterations)
        
        self.reset(self.controller.reset_ar())
        self.ani = FuncAnimation(self.figure, self.update, frames=frames, interval=self.period_ms, repeat=False, event_source=self.event_source)
        self.figure.show()
        
        return self.state_posterior
            
        
    def reset(self, reset_arr):
        '''
        This methods sets the initial condition of state and time and is called prior to each simulation.
        
        Example:
            self.time = 0.0
            self.state = [0.0 , 0.0]
        '''
        raise NotImplementedError("This method is just an interface. Implement in inheriting class")
    
    def update(self, iteration_number):
        '''
        This method runs the simulation steps by calling:
            - The control method to generate control input
            - The model method for updating the state
            - The draw method to update the figure
            - The upkeep method to record data and control the simulation
            
        Arguments:
            iteration_number [] : The iteration number.
        '''
        self.iteration = iteration_number
        self.time = self.time + self.period_s
        self.control()
        self.model()
        self.draw()
        self.upkeep()

    def control(self):
        '''
        This method handles the control algorithm. Must return input vector for model
        '''
        pass
       
    def model(self):
        '''
        This method calculates the new state of the system. Must return new state.
        '''
        raise NotImplementedError("This method is just an interface. Implement in inheriting class")

    def draw(self):
        '''
        This method handles updating or redrawing the figure
        '''
        raise NotImplementedError("This method is just an interface. Implement in inheriting class")    

    def upkeep(self):
        '''
        This method handles saving data for later plotting or analysis
        '''
        pass
    
    def stop(self):
        '''
        This method stops the animation 
        '''
        self.ani._stop() 
        
    
class CartPoleSimulation(Simulation):
    '''
        Implements the cart-pole simulation based on the equations of motion derived in:
        http://ocw.mit.edu/courses/electrical-engineering-and-computer-science/6-832-underactuated-robotics-spring-2009/readings/MIT6_832s09_read_ch03.pdf
        
        The simulation will reset if the angle exceeds max_angle, if the position exceeds max_deviation or if the duration is more than max_time. (This has been changed, so that it is dictated from the controllers)
        The simulation will run a number of times corresponding to the variable trials - 1
    '''
    def __init__(self, *args, **kwargs):
        super( CartPoleSimulation, self ).__init__(*args, **kwargs)
        
        # Constants
        self.cart_width = 1.0
        self.cart_height = 0.5
        self.cart_mass = 10.0
        self.pendulum_length = 1.0
        self.pendulum_mass = 1.0
        self.gravity = -9.82
        self.max_time = 10

        self.obst_pos = [-1.5,1]

        # Setup figure
        self.cart = Rectangle([-self.cart_width/2, -self.cart_height/2], self.cart_width, self.cart_height)
        self.axes.add_patch(self.cart)
        (self.pendulum,) = self.axes.plot([], [], 'o-', lw=2)
        self.obstacle = Circle(self.obst_pos,0.04)
        self.axes.add_patch(self.obstacle)

        # Setup simulation
        self.trial = 0
        self.trials = 100 # TODO: Change implementation to fit this number instead of this number - 1
        self.state_posterior = [[]]*(self.trials)
        
    def reset(self, reset_arr):
        self.time = 0.0
        self.state = [reset_arr[0] , reset_arr[1], reset_arr[2], reset_arr[3]] # angle, position, angular_velocity, velocity
        self.input = [0.0]
        self.trial += 1
    
    def model(self):
        # Update accelerations
        acceleration = 1/(self.cart_mass+self.pendulum_mass*sin(self.state[0])**2)*(self.input[0]+self.pendulum_mass*sin(self.state[0])*(self.pendulum_length*self.state[2]**2+self.gravity*cos(self.state[0])))
        angular_acceleration = 1/(self.pendulum_length*(self.cart_mass+self.pendulum_mass*sin(self.state[0])**2))*(-self.input[0]*cos(self.state[0])-self.pendulum_mass*self.pendulum_length*self.state[2]**2*cos(self.state[0])*sin(self.state[0])-(self.cart_mass+self.pendulum_mass)*self.gravity*sin(self.state[0]))
        # Update velocities
        self.state[3] = self.state[3] + acceleration*self.period_s 			#cart
        self.state[2] = self.state[2] + angular_acceleration*self.period_s 	#pole
        
        # Update angle and position
        self.state[0] = self.state[0] + self.state[2]*self.period_s 	#pole
        self.state[1] = self.state[1] + self.state[3]*self.period_s		#cart

    def draw(self):
        Rectangle.set_xy(self.cart, [ self.state[1]-self.cart_width/2 , -self.cart_height/2 ])
        self.pendulum.set_data([(self.pendulum_length-0.05)*sin(self.state[0])+self.state[1],self.state[1]], [(self.pendulum_length-0.05)*cos(self.state[0]), 0])

    def control(self):
        self.controller.update( self.state )
        self.input[0] = self.controller.control_output[0]

    def upkeep(self):
        # if the test does not fail within a specific time
        if self.time > self.max_time :
            self.controller = self.controllers_arr[0]
            self.reset(self.controller.reset_ar())

        # if the cart is past the obstacle
        if self.controller == self.controllers_arr[1] and self.state[1] > self.obst_pos[1]:
            self.controller = self.controllers_arr[2]

        # if the pole is under the obstacle
        if self.controller == self.controllers_arr[0] and self.state[0] > self.controller.reference_p_ang:
            self.controller = self.controllers_arr[1]

        # if the system has failed, reset the system
        if self.controller.out_of_bounds(self.state):
            self.controller = self.controllers_arr[0]
            self.reset(self.controller.reset_ar())
            print "reset" # for debug

           
        #if self.state[0] > self.controller.res_angle or self.state[0] < -self.controller.res_angle or self.state[1] > self.controller.res_deviation or self.state[1] < -self.controller.res_deviation:
           
        if self.trial > self.trials-1:
            self.stop()
        else :
            self.state_posterior[self.trial-1].append(copy(self.state)+self.input)


class Controller(object):
    '''
    Implements the interface for a controller.
    
    The simulation object will call the update method once for every state update and read the control_output variable
    '''
    def __init__(self):
        self.control_output = [0.0]
    
    def update(self, state):
        pass
    
class ANNController(Controller):
    def __init__(self):
    	super( ANNController, self ).__init__()

        self.reset_arr = []

    	# Initialization for ANN
    	self.reflex_angle = 0 #deg2rad(10)
    	self.rew_angle = 0 #deg2rad(12)
        self.reflex_deviation = 0 #2.0
        self.rew_deviation = 0 # 2.2
        self.mew = 0.1
        self.gain = 11.0
        self.reflex_gain = 1.0
        
        self.weight_gains = matrix([1.0 for i in range(4)])
        self.weights 	  = matrix([0.0 for i in range(4)]) #pole, cart, pole velocity, cart velocity
        self.weights 	  = r_[self.weights,self.weights]
        self.temp_state	  = matrix([0.0 for i in range(4)])
        self.Z			= 0

        #constants for the out of bounds function that is used to reset the simulation
        #and for the reward function
        self.res_angle = self.rew_angle + deg2rad(2)
        self.res_deviation = self.rew_deviation + 0.2

        # Reflex
        self.reflex_sig = 0

        # Reward
        self.reward_sig = 0
        self.reward_sig_old = 0

    def inputNormalize(self, state):
    	self.temp_state[0,0] = (state[0] - self.reference_p_ang) / deg2rad(2)
    	self.temp_state[0,1] = (state[1] - self.reference_c_pos) / 2.4
    	self.temp_state[0,2] = (state[2] - self.reference_p_vel)
    	self.temp_state[0,3] = (state[3] - self.reference_c_vel) / 3

    def ANN(self):
    	self.Z = tanh(self.temp_state * transpose(self.weights))
    	self.Z = float(self.Z[0])
    	print self.Z
    	
    	#self.Z =  tanh((state[0] - self.reference_p_ang) / deg2rad(12)) * self.weights[0] # pole
    	#self.Z += tanh((state[1] - self.reference_c_pos) / 2.4) * self.weights[1] # cart
    	#self.Z += tanh(state[2] - self.reference_p_vel) * self.weights[2] # pole speed
    	#self.Z += tanh((state[3] - self.reference_c_vel) / 5) * self.weights[3] # cart speed
    	#self.Z = tanh(self.Z)
    	
    def reflex(self, state):
    	if (state[0] > self.reflex_angle) or (state[1] < -self.reflex_deviation):
    		reflex_sig = 1
    	elif (state[0] < -self.reflex_angle) or (state[1] > self.reflex_deviation):
    		reflex_sig = -1
    	else :
    		reflex_sig = 0

    def reward(self, state):
        if state[0] > (self.reference_p_ang + self.rew_angle) or state[0] < (self.reference_p_ang - self.rew_angle) or state[1] > (self.reference_c_pos + self.rew_deviation) or state[1] < (self.reference_c_pos - self.rew_deviation):
            #self.weights += self.mew * abs( self.temp_state) * transpose(self.weight_gains)
            self.weights[0,0] += self.mew * abs( self.temp_state[0,0]) * self.weight_gains[0,0]
            self.weights[0,1] += self.mew * abs( self.temp_state[0,1]) * self.weight_gains[0,1]
            self.weights[0,2] += self.mew * abs( self.temp_state[0,2]) * self.weight_gains[0,2]
            self.weights[0,3] += self.mew * abs( self.temp_state[0,3]) * self.weight_gains[0,3]

    def out_of_bounds(self, state): #called from simulation to determine if the simulation should reset
        if state[0] > (self.reference_p_ang + self.res_angle) or state[0] < (self.reference_p_ang - self.res_angle) or state[1] > (self.reference_c_pos + self.res_deviation) or state[1] < (self.reference_c_pos - self.res_deviation):
            return True
        else:
            return False

    def update(self, state):
    	self.inputNormalize(state)
    	self.reward(state)
    	self.ANN()
        self.reflex(state)
        self.control_output[0] = self.reference_output + self.Z * self.gain + self.reflex_sig * self.reflex_gain 

class ANNController_balance(ANNController):
    def __init__(self):
        super( ANNController_balance, self ).__init__()

        # Initialization for ANN
        self.reflex_angle = deg2rad(12)
        self.rew_angle = deg2rad(14)
        self.reflex_deviation = 1.0
        self.rew_deviation = 2.0
        self.mew = 0.05
        self.gain = 150.0
        self.reflex_gain = 10.0

        #reference values
        self.reference_p_ang = 0
        self.reference_c_pos = 0.0
        self.reference_p_vel = 0
        self.reference_c_vel = 0
        self.reference_output= 0

        #constants for the out of bounds function that is used to reset the simulation
        #and for the reward function
        self.res_angle = self.rew_angle + deg2rad(2)
        self.res_deviation = self.rew_deviation + 0.2
    
    def reset_ar(self): #this needs to be updated as there is a random parameter in the array
        random_value = (rand()-0.5)*10
        while abs(random_value) < 1:
            random_value = (rand()-0.5)*10
        reset_arr = [deg2rad(random_value), -1.0, 0.0 , 0.0] #angle, pos, ang vel, cart vel
        return reset_arr

class ANNController_move(ANNController):
    def __init__(self):
        super( ANNController_move, self ).__init__()

        self.first = True

        # Initialization for ANN
        self.reflex_angle = deg2rad(0.5)
        self.rew_angle = deg2rad(1)
        self.reflex_deviation = 1.0
        self.rew_deviation = 3.0
        self.gain = 50.0
        self.reflex_gain = 0.0

        self.weight_gains[0,1] = 0.0
        self.weight_gains[0,3] = 0.0
        self.weight_gains[0,2] = 4.0

        self.reference_p_ang = deg2rad(11.4783 + 1)
        self.reference_c_pos = -2
        self.reference_p_vel = 0
        self.reference_c_vel = 0
        self.reference_output= 23.9045

        self.res_angle = self.rew_angle + deg2rad(1)
        self.res_deviation = self.rew_deviation + 0.2
        self.obst_angle = deg2rad(11.4783) 
    
    def reset_ar(self): #this needs to be updated as there is a random parameter in the array
        reset_arr = [deg2rad((rand()-0.5)) + self.reference_p_ang, -2.0, .626738, 0.0]
        self.first = True
        return reset_arr

    def out_of_bounds(self, state): #called from simulation to determine if the simulation should reset
        if state[0] > (self.reference_p_ang + self.res_angle) or state[0] < (self.reference_p_ang - self.res_angle) or state[0] < self.obst_angle: # or state[1] > (self.reference_c_pos + self.res_deviation) or state[1] < (self.reference_c_pos - self.res_deviation) or state[0] < self.obst_angle:
            return True
        else:
            return False

class ANNController_to_angle(ANNController):
    def __init__(self):
        super( ANNController_to_angle, self ).__init__()

        self.weight_gains[0,1] = 0.0
        self.weight_gains[0,3] = 0.0
        self.weight_gains[0,2] = 4.0

        # Initialization for ANN
        self.rew_angle = deg2rad(12)
        self.reflex_deviation = 1.0
        self.rew_deviation = 1.5
        self.rew_p_vel = 0.2
        self.gain = 1


        self.reference_p_ang = deg2rad(11.4783)
        self.reference_c_pos = -2.0
        self.reference_p_vel = 0
        self.reference_c_vel = 0
        self.reference_output= 0

        self.res_angle = self.rew_angle + deg2rad(1)
        self.res_deviation = self.rew_deviation + 0.2
        self.res_p_vel = self.rew_p_vel + 0.02
    
    def reset_ar(self): #this needs to be updated as there is a random parameter in the array
        reset_arr = [0.0001, -2.9, 0.0, 0.0]
        return reset_arr

    def out_of_bounds(self, state): #called from simulation to determine if the simulation should reset
        if state[0] > (self.reference_p_ang + self.res_angle) or state[0] < (self.reference_p_ang - self.res_angle):
            return True
        else:
            return False

    def reflex(self, state):
    	reflex_sig = 0
        
if __name__ == '__main__':
    # Instantiate simulation
    sim = CartPoleSimulation(figsize=(9,2.25), xlim=(-3,3), ylim=(-0.25,1.25))
    
    # Set the controller
    sim.controller = ANNController_to_angle()
    sim.controllers_arr[0] = ANNController_to_angle()
    sim.controllers_arr[1] = ANNController_move()
    sim.controllers_arr[2] = ANNController_balance()
    #sim.controller.resetArr(1)
    
    # Run simulation
    posterior = sim.startAnimation()

    # Show state posterior of last run ( posterior[-1] )
    clf()
    time_line = [i*sim.period_s for i in range(len(posterior[-1]))]

    print "pole weight"
    print sim.controllers_arr[1].weights[0,0]
    print "cart weight"
    print sim.controllers_arr[1].weights[0,1]
    print "pole speed weight"
    print sim.controllers_arr[1].weights[0,2]
    print "cart speed weight"
    print sim.controllers_arr[1].weights[0,3]
    
    print "pole weight"
    print sim.controllers_arr[2].weights[0,0]
    print "cart weight"
    print sim.controllers_arr[2].weights[0,1]
    print "pole speed weight"
    print sim.controllers_arr[2].weights[0,2]
    print "cart speed weight"
    print sim.controllers_arr[2].weights[0,3]

    plot(time_line, posterior[-1])
    legend(['Angle','position','angular velocity', 'linear velocity', 'control signal'])
    show()