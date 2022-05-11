import numpy as np
import cv2
from time import time
import argparse

## ----------------------------------------------------------------------------
''' MAIN CODE '''
if __name__ == "__main__":
    # Connects to your computer's default camera
    cap = cv2.VideoCapture(0)

    '''
    Code from:
    https://docs.opencv.org/3.4/d5/d6f/tutorial_feature_flann_matcher.html


    Transformation matrix for matching sift points:
    https://webpages.charlotte.edu/jfan/matching.pdf
    '''


    img1 = cv2.imread("Bootlegger.jpg")
    # new_shape = (295,568) #Taille de l'image de référence
    # img1 = cv2.resize(img1, new_shape)


    #For effiency I calculate the descriptor of the reference image only once
    sift = cv2.SIFT_create()
    keypoints1, descriptors1 = sift.detectAndCompute(img1,None) #Ref


    while True:
        ret, img2 = cap.read()

        #-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
        keypoints2, descriptors2 = sift.detectAndCompute(img2,None)

        #-- Step 2: Matching descriptor vectors with a FLANN based matcher
        matcher = cv2.DescriptorMatcher_create(cv2.DescriptorMatcher_FLANNBASED)
        knn_matches = matcher.knnMatch(descriptors1, descriptors2, 2)
        
        #-- Filter matches using the Lowe's ratio test
        ratio_thresh = 0.5
        good_matches = []
        for m,n in knn_matches:
            if m.distance < ratio_thresh * n.distance:
                good_matches.append(m)

        #-- Draw matches
        img_matches = np.empty((max(img1.shape[0], img2.shape[0]), img1.shape[1]+img2.shape[1], 3), dtype=np.uint8)


        #------------ Fonction to get the position of the good matches
        # Initialize lists
        list_kp = []
        # For each match...
        for mat in good_matches:
            # Get the matching keypoints for each of the images
            idx = mat.trainIdx

            # x - columns
            # y - rows
            (x, y) = keypoints2[idx].pt

            # Append to each list
            list_kp.append((x,y))


        # ---------- Draw the good matches
        img_kp = img2.copy()
        for x,y in list_kp:
            img_kp = cv2.circle(img_kp, (int(x),int(y)), 2, (255,0,255), 5)



        #-- Show detected matches
        cv2.imshow('Good Matches', img_kp)


        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
