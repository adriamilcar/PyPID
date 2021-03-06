# Python-PID
Python implementation of a PID controller.

Libraries needed to use this module: numpy and matplotlib. Both can be installed via "pip install" through the command line.

The module is highly intuitive and has been designed for high-level usage. The user only has to declare the PID object, along with the reference value, and the corresponding gains (proportional, derivative and integral): 
PID = PID(ref, pGain, dGain, iGain). 

Then, for each iteration, the new input is passed through an update function, which in turn returns the output of the controller:
output=PID.update(input). 

The reference value and the gains can also be changed at any point in time: 
e.g. PID.change_reference(new_reference) or PID.change_proportional_gain(new_gain). 

Finally, the results (time series of the output and errors) can be easily plotted: 
PID.plot_error() and PID.plot_output(). Moreover, PID.plot_error() allows to select if one wants to plot only the error, the differential error (rate of change, or derivative), and/or the accumulated error. E.g. PID.plot.error(["proportional", "derivative"]) will plot the error through time, as well as its derivative.

Therefore, it is quite straightforward to integrate this PID module in more complex systems, like PID cascades, or feed-forward + feedback systems. Also, it allows for adaptive control schemes since the gains are parameters that can be fitted as well.



In the following days I'll also add an example of how to control a cart pole system using two PIDs running in parallel. Specifically I'll show that (1) the system maintains the cart pole stable at a specific position; (2) it can tolerate a certain amount of disturbances; and (3) it can smoothly change from one position to another without destabilizing and in a decent amount of time.
