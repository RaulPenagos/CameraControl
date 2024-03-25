from statsmodels.base.model import GenericLikelihoodModel
import numpy as np
import matplotlib.pyplot as plt

from src.Robot import Robot

class MyLikelihood(GenericLikelihoodModel):

     def __init__(self, endog, exog, robot, **kwds):
         
        self.robot = robot
        super(MyLikelihood, self).__init__(endog, exog, **kwds)

     def loglike(self, params):

        self.robot.param = params
                


         return -chi2






##################################################################################################
########################################### Main #################################################
##################################################################################################
if __name__ == "__main__":


    #Let's define the number of points in our example and the range of the fit
    npoints = 100
    xmin = -10
    xmax = 10
    
    #Let's use the MyLikelihood to fit a third order polynom a * x^3 + b * x^2 + c * x + d 
    a = -1
    b = -1
    c = -1
    d =  1

    #Error in our measurements
    error = 0.1
    
    #Variable x will be our independent coordinate (it should be r in your example)
    x = np.random.uniform(xmin, xmax, npoints)

    #Variable y will be our dependent coordinate (it should be theta in your example)
    y = a * x * x *x + b * x * x + c * x + d

    #We will add an uncertainty to our dependent coordinate to account for the error
    y = y + np.random.normal(0, error, npoints)

    #At this point we have the pair (x, y) that contains the independent and the dependent variables
    #
    #In order to use the MyLikelihood we need to identify the different arguments:    
    #
    #endog is the dependent variable y
    #
    #exog is the independent variable x
    #
    #sigmad is the error in the measurement 
    #
    #params is the list of parameters to find: a, b, c and d
    l = MyLikelihood(y, x, error)
    
    #Now we fit the model
    result = l.fit(start_params=[0, 0, 0, 0])
    
    print('a: ', result.params[0]) 
    print('b: ', result.params[1]) 
    print('c: ', result.params[2]) 
    print('d: ', result.params[3]) 
   
       
    poly = np.poly1d(result.params)
    new_x = np.linspace(min(x), max(x))
    new_y = poly(new_x)
    plt.plot(x, y, 'ro', new_x, new_y)
    plt.ylabel('y')
    plt.xlabel('x')
    plt.show()





