import numpy as np
import matplotlib.pyplot as plt
# Only the numpy and matplotlib libraries are needed for using this PID module.


'''
 PID class that allows the instantiation of a very modular discrete-time PID object, to be used or inherited by other programs or classes.
 In order to have P, PD, or PI controllers (instead of a complete PID), the corresponding gains should be set to 0 when declaring the object.
'''
class PID(object):

    # Instantiates the object with the user-defined gains and value of reference, and also prepares the lists/variables to keep track of everything.
    def __init__(self, reference, p_gain, d_gain, i_gain):

        self.pGain = p_gain    # Proportional gain.
        self.dGain = d_gain    # Derivative gain.
        self.iGain = i_gain    # Integral gain.
        self.ref = reference   # Reference value.
        self.outputs = [0.]    # PID's output through time.
        self.errors = [0.]     # Error through time.
        self.acc_err = 0       # Accumulated error for computing the integral term.
        self.time = 0.         # Current time of simulation.
        self.dt = 1.           # Magnitude of increment of time in each iteration.


    # It takes the current input and generates the error, given the current reference value, and outputs.
    # The value of the weighted sum of the three terms (proportional, derivative, integral).
    def update(self, input):

        error = self.ref - input      # Calculate the error subtracting the current input from the current reference value.
        self.errors.append(error)     # Stores the current error to be plotted at the end as a time series.
        self.acc_err += error         # Updates the accumulated error with the current error.

        # The output is calculated as the weighted sum of the three terms with their respective gains,
        # using the current error (proportional term); the difference between the current and the previous error (derivative);
        # and the accumulated error (integral), in this order.
        out = self.errors[-1] * self.pGain + \
              (self.errors[-1]-self.errors[-2]) * self.dGain + \
              self.acc_err * self.iGain

        self.outputs.append(out)      # Stores the current output to be plotted at the end as a time series.

        self.time += self.dt          # Increases the time step by one "dt".

        return out                    # Returns the output of the PID.


    # It plots (1) the error itself through time; (2) the derivative of the error (rate of change through time);
    # (3) the accumulated error through time; (4) or the three of them (in the same figure).
    # In order to select which error/s to plot, the function takes as an input a list with the names of the errors:
    # "proportional", "derivative", or/and "integral" (or nothing to plot them all). By default it plots the three of them.
    def plot_error(self, type_err=[]):

        # Plots the temporal evolution of the error itself.
        if any('proportional' in e for e in type_err):
            plt.plot(self.errors, label='Error')
            # The list to be plotted is just the list of all the errors that have been stored through time.

        # Plots the temporal evolution of the error's rate of change.
        elif any('derivative' in e for e in type_err):
            plt.plot(np.append(0, [y-x for x,y in zip(self.errors,self.errors[1:])]), label='Differential error (derivative)')
            # The list to be plotted is created by subtracting each element in self.errors by its previous element in self.errors.

        # Plots the temporal evolution of the accumulated error
        elif any('integral' in e for e in type_err):
            plt.plot(np.cumsum(self.errors), label='Accumulated error (integral)')
            # The list to be plotted is created by, for each element in self.errors, summing over all the previous elements in self.errors.

        # Plots the temporal evolution of the error, its rate of change, and its accumulation (all the previous plots in one).
        else:
            plt.plot(self.errors, label='Error')
            plt.plot(np.append(0., [y - x for x, y in zip(self.errors, self.errors[1:])]), label='Differential error (derivative)')
            plt.plot(np.cumsum(self.errors), label='Accumulated error (integral)')

        plt.hlines(0., 0, len(self.errors), linestyles='dashed')  # Draws a dashed line at 0, as a reference of performance (minimizing the error).
        plt.title('Error of PID controller')                      # Sets the title of the plot.
        plt.xlabel("Timestep")                                    # Sets the label for the x axis.
        plt.ylabel("Error")                                       # Sets the label for the y axis.
        plt.legend()                                              # Draws the legend with the names of the corresponding errors.
        plt.show()                                                # Plots the final figure.


    # Plots the output of the PID controller through time
    def plot_output(self):

        plt.plot(self.outputs, label='Output')     # Plots the outputs that have been stored through time.
        plt.title('Output of PID controller')      # Sets the title of the plot.
        plt.xlabel("Timestep")                     # Sets the label for the x axis.
        plt.ylabel("Output")                       # Sets the label for the y axis.
        plt.show()                                 # Plots the final figure.


    # Changes the current reference value of the PID to the new one: "new_ref".
    def change_reference(self, new_ref):

        self.ref = new_ref         # Sets the new reference value.


    # Changes the current gain (weight) of the PID's proportional term, to the new one: "new_gain".
    def change_proportional_gain(self, new_gain):

        self.pGain = new_gain      # Sets the new proportional gain.


    # Changes the current gain (weight) of the PID's derivative term, to the new one: "new_gain".
    def change_derivative_gain(self, new_gain):

        self.dGain = new_gain      # Sets the new derivative gain.


    # Changes the current gain (weight) of the PID's integral term, to the new one: "new_gain".
    def change_integral_gain(self, new_gain):

        self.iGain = new_gain      # Sets the new integral gain.


    # Returns the current time of the simulation (which equals to: number_of_iterations * dt).
    def get_time(self):

        return self.time           # Gets the current time.
