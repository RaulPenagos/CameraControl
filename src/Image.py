import numpy as np
from matplotlib import pyplot as plt
from matplotlib.image import imread
import os # path to the project, used in Image.save()
import datetime as dt


from scipy.optimize import minimize 
from statsmodels.base.model import GenericLikelihoodModel

"""
Script defining Camera and CircleCompleter class. Enables the processing of taken images.
Using CircleCompleter, it enables to calculate the center of circles given an image
of just a portion.

Author: Raul Penagos
Date: Mar 22th, 2025
"""


class Image:
    """
    Class Image, helps in formating and processing pictures taken with an IDS camera.
    Methods of the class:
    save()    display()   binarize()  soften()    find_cm()   search_border()  circle_treatment()

    Para facilitar la minimización definir unos valores iniciales del centro acorde con lo esperado
    """
    def __init__(self, matrix, save_path: str = './img'):
        """
        Input
            matrix: matriz de la imagen
            save_path: path relativo a la posición de este script donde se quiere guardar
        """
        self.image = np.copy(matrix[:,:,0])
        self.image_original = np.copy(matrix)
        self.timestamp = dt.datetime.now()
        self.file_name = save_path + f'/img_{self.timestamp.strftime("%Y%m%d_%H%M%S")}.png'

        dirname = os.path.dirname(__file__)  # Edit where to save the img
        self.abs_filename = os.path.join(dirname, self.file_name)

        self.cm = (None, None)


    def save(self):
        """
        Save the image to the file name given by the Image's time stamp.
        Make sure the folder exists.
        """
        plt.figure(figsize = (5,5))        
        plt.savefig(self.abs_filename, bbox_inches='tight', pad_inches=0)

    def display(self, save = False):
        """
        Show and optionally save the image on screen.
        Shows CM if it has been computed.
        """
        plt.close("all")
        fig, ax = plt.subplots(figsize=(5, 5))
        ax.imshow(self.image, cmap='gray')
        ax.axis('off')  # Oculta ejes y ticks

        if self.cm != (None, None):
            ax.plot(self.cm[0], self.cm[1], 'or')

        if save:
            plt.savefig(self.abs_filename, bbox_inches='tight', pad_inches=0)

        plt.show()

    def binarize(self, tresh: float = 2):
        """
        Binarize the image to (0, 255) grey scale (black and white).
        """
        self.image = np.where(self.image > self.image.max()/tresh, 255, 0)
        return self
    
    def soften(self, m = 8 ):
        """
        Soften filter smoothens the image by taking an average of a m size
        square kernel.
        """
        # Average filter
        for i in range(0+m, self.image.shape[0] - m):
            for j in range(0+m, self.image.shape[1] - m):
                self.image[i,j] = np.mean(self.image[i-m:i+m, j-m:j+m])
        return self
    
    def find_cm(self):
        """
        Given a binarized picture, it subtracts the center of fiducials (white) 
        on a black Background.
        """
        # Average of white Pixels, for binarized pictures 
        yy, xx = np.where(self.image > 0)
        # self.cm = np.array([xx, yy])
        self.cm = (xx.mean(), yy.mean()) if xx.size > 0 else (None, None)
             
    def search_border(self, show = True, save = False):
        """
        Busca los pixels de frontera en una imagen binarizada, aquellos que estén rodeados 
        por pixels de distinto color. (up, down, right, left)
        """
        # Defino frontera
        y_max , x_max = np.asarray(self.image.shape) - 1

        # Desplazamientos en las 4 direcciones principales
        up    = np.roll(self.image, shift=-1, axis=0)
        down  = np.roll(self.image, shift=1, axis=0)
        left  = np.roll(self.image, shift=1, axis=1)
        right = np.roll(self.image, shift=-1, axis=1)

        print(self.image.max())
        print(self.image.min()) 

        # Detectar bordes: puntos donde hay un 1 y algún vecino es 0
        # border_mask = (self.image == 1) & ((up == 0) | (down == 0) | (left == 0) | (right == 0))
        border_mask = (self.image == 0) & ((up > 0) | (down > 0) | (left > 0) | (right > 0))

        # Obtener coordenadas de los bordes
        y, x = np.where(border_mask)

        # Quito elementos de los bordes de imagen
        index_x = np.copy([i for i, xx in enumerate(x) if (xx <= 1 or xx >= x_max)])
        if len(index_x) != 0:
            y = np.delete(y, index_x)
            x = np.delete(x, index_x)

        index_y = np.copy([i for i, xx in enumerate(y) if (xx <= 1 or xx >= y_max)])
        if len(index_y) != 0:
            y = np.delete(y, index_y)
            x = np.delete(x, index_y)

        if show:
            plt.plot(x, y, 'ro') 
            plt.imshow(self.image, cmap = 'gray')   
            if save:
                plt.savefig('./img/tfg_test/border.png', bbox_inches='tight', pad_inches=0)
            plt.show()

        return x, y
    
    def circle_treatment(self, show = True, save = False):
        """
        Given the coordinates x,y of a circles border, or segment of its border
        It will minimize through a Likelihood function the center (a,b) of the 
        circle and its radius (r).
        Returns: 
            a: posición centro en x
            b: posición centro en y
        """
        x, y = self.search_border()

        cal = CircleFit(x, y)

        results = cal.fit()    

        print(results.summary())

        print(results.params)

        a, b, r = results.params

        if show:
            plt.imshow(self.image, cmap = 'gray')   
            plt.scatter(a,b, s = 30, c = 'b')   
            if save:
                plt.savefig('./img/tfg_test/circ_center.png', bbox_inches='tight', pad_inches=0)
            plt.show()


        return a,b


class CircleFit(GenericLikelihoodModel):
    """
    Clase Likelihood para crear modelos a partir de los puntos (x, y)
    del borde de un círculo y obtener mediante una minimización la posición
    del centro (x_c, y_c) y el radio r del círculo que mejor se ajusta a los datos.
    """

    def __init__(self, exog, endog, **kwds):
        """
        exog (array):  X, coordenadas x de los puntos del borde del círculo
        endog (array): Y, coordenadas y de los puntos del borde del círculo
        """
        self.n = int(len(exog))
        
        self.exog = np.asarray(exog)
        self.endog = np.asarray(endog)
        print(self.exog)
        print(self.endog)
        # Se dan valores iniciales razonables a los parámetros-----------------
        self.a = np.mean(exog)  # Posición centro en x
        self.b = np.mean(endog)  # Posición centro en y
        self.r = (np.max([(max(self.exog)-min(self.exog)), (max(self.endog)-min(self.endog))]))  # Radio círculo

        super(CircleFit, self).__init__(endog, exog, **kwds)  


    def loglike(self, params):
        #  Se actualizan los parámetros
        self.a = params[0]
        self.b = params[1]
        self.r = params[2]

        chi2 = 0.0

        for i in range(0, self.n):
            chi2 += np.abs(((self.exog[i]-self.a)**2 + (self.endog[i]-self.b)**2 - self.r**2))
        print('CHI2: ', chi2)

        return -chi2

    def fit(self, start_params=None, method='powell', maxiter=10000, **kwargs):
        # method = nm, ha funcionado 

        if start_params is None:
            start_params =  [self.a, self.b, self.r]
        return super(CircleFit, self).fit(start_params=start_params, method=method, maxiter=maxiter, **kwargs)




def test7():
    fig = Image(cv2.imread('./img/tfg_test/fiducial.png'))
    
    fig.binarize().display(True)
    fig.soften(8).display()
    fig.find_cm()
    fig.display(True)

def test8():
    fig = Image(cv2.imread('./img/tfg_test/circ1.png'))
    # fig.binarize().display()
    fig.soften(2).binarize(1.5).display(True)
    fig.search_border(show=True, save=True)
    fig.circle_treatment(show=True, save=True)

def test9():
    fig = Image(cv2.imread('./img/tfg_test/circ2.png'))
    # fig.binarize().display()
    fig.soften(16).binarize(2.5).display(True)
    fig.search_border(show=True, save=True)
    fig.circle_treatment(show=True, save=True)



def main():
    
    import cv2
    test9()
    
      
   



if __name__ == "__main__":

    main()

    

    

