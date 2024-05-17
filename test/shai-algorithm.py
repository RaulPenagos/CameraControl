# organizing imports 
import cv2 
import numpy as np 



def shi_tomasi(image):

    #Converting to grayscale
    gray_img = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    
    #Specifying maximum number of corners as 1000
    # 0.01 is the minimum quality level below which the corners are rejected
    # 10 is the minimum euclidean distance between two corners
    corners_img = cv2.goodFeaturesToTrack(gray_img,4000,0.03,0.1)
    
    corners_img = np.int0(corners_img)

    for corners in corners_img:
       
        x,y = corners.ravel()
        #Circling the corners in green
        cv2.circle(image,(x,y),3,[0,255,0],-1)

    return image

if __name__=='__main__':

    #camera = cv2.VideoCapture(0)
    #return_value, image = camera.read()
    image = cv2.imread('module.jpeg') 
    corners_image = shi_tomasi(image)
    cv2.imwrite('picture.png', image)  
