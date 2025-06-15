# organizing imports 
import cv2 
import numpy as np 


camera = cv2.VideoCapture(0)

return_value, image = camera.read()
# path to input image specified and  
# image is loaded with imread command 
#image = cv2.imread('caca.jpeg') 
  
# convert the input image into 
# grayscale color space 
operatedImage = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) 
  
# modify the data type 
# setting to 32-bit floating point 
operatedImage = np.float32(operatedImage) 
  
# apply the cv2.cornerHarris method 
# to detect the corners with appropriate 
# values as input parameters 
dest = cv2.cornerHarris(operatedImage, 2, 5, 0.07) 
  
# Results are marked through the dilated corners 
dest = cv2.dilate(dest, None) 
  
# Reverting back to the original image, 
# with optimal threshold value 
image[dest > 0.05 * dest.max()]=[0, 0, 255] 
  
# the window showing output image with corners 
#cv2.imshow('Image with Borders', image) 
cv2.imwrite('picture.png', image)  
