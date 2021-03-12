#!/usr/bin/env python
import rospy
import sys
import os
import math
import numpy as np
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

role_name = 'brandons_ride'
image_topic_name = "/carla/{}/rgb_front/image".format(role_name)
vehicle_status_topic_name = "/carla/{}/vehicle_status".format(role_name)

IMG_WIDTH = 800
IMG_HEIGHT = 600

frame = None
bridge = CvBridge()

def image_callback(ros_image):
    global frame, IMG_HEIGHT, IMG_WIDTH
    frame = bridge.imgmsg_to_cv2(ros_image, "bgr8")

LINE_COLOR = (0, 255, 0)
LANE_COLOR = (0, 0, 255)
LANE_REGION_COLOR = (0, 255, 0)
LANE_CENTER_COLOR = (180, 0, 0)
CAR_CENTER_COLOR = (180, 180, 0)

def adjust_gamma(image, gamma=1.0):
    invGamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** invGamma) * 255
        for i in np.arange(0, 256)]).astype("uint8")
    return cv2.LUT(image, table)

def mask_edges(img):
    ignore_mask_color = 255
    mask = np.zeros_like(img)    
    imshape = img.shape

    #vertices of a mask polygon
    lower_left = [0,imshape[0]]
    lower_right = [imshape[1], imshape[0]]
    top_left  = [0, 0]
    top_right = [imshape[1], 0]
    top_middle = [imshape[1]/2, 0] 
    center = [imshape[1]/2, imshape[0]/2]

    vertices = [np.array(
        [lower_left, [imshape[1]/2-100, 0], [imshape[1]/2+100, 0] , lower_right], 
        dtype=np.int32)]

    cv2.fillPoly(mask, vertices, ignore_mask_color)

    copied_img =  np.copy(img)
    pts = np.array([vertices], np.int32)
    pts = pts.reshape((-1,1,2))
    cv2.polylines(copied_img, [pts], True, (255,255,255))
    # cv2.imshow('mask region', copied_img)
    
    img = cv2.bitwise_and(img, mask)
    return img


def warpPerspective(img, inverse=False):
    #perspective transformation of img
    WARP_START_Y = 380
    src = np.float32([[0, WARP_START_Y], [IMG_WIDTH, WARP_START_Y], [IMG_WIDTH,IMG_HEIGHT], [0,IMG_HEIGHT]])        
    dst = np.float32([[0,0], [IMG_WIDTH,0], [IMG_WIDTH,IMG_HEIGHT], [0,IMG_HEIGHT]])
    if inverse==True:
        tmp = dst
        dst = src
        src = tmp
    
    M = cv2.getPerspectiveTransform(src, dst)    
    trans = cv2.warpPerspective(img,M,(IMG_WIDTH,IMG_HEIGHT))
    
    return trans

prev_cte = 0
def detect(org_image):
    """
    Change gamma
    Increase sensitivity for straight lines in Canny (more combined lines)
    Longer lines filter in Hough
    """
    global prev_cte
    print('========================')
    
    # cv2.imshow('1 original image', org_image)

    #Gamma correction
    img = org_image
    img = adjust_gamma(img, .17)
    # cv2.imshow('2 after adjust_gamma', img)
    
    #Perspective transformation
    img = warpPerspective(img)        
    transformed_color_img = img
    #cv2.imshow('3 after warp perspective', transformed_color_img)

    # hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
    # _, s_threshold = cv2.threshold(hls[:,:,2], 80, 255, cv2.THRESH_BINARY)
    # #cv2.imshow('sat thresh', s_threshold)

    # _, r_threshold = cv2.threshold(img[:,:,2], 180, 255, cv2.THRESH_BINARY)
    # cv2.imshow('red thresh', r_threshold)
        
    #Conversion to a grayscale image
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cv2.imshow('4 grayscale', img)

    #thresholding
    after_thresholding = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 21, 10)
    cv2.imshow('5 after thresholding', after_thresholding)
    img = after_thresholding

    """_, after_thresholding = cv2.threshold(img, 75, 255, cv2.THRESH_BINARY) # TODO: Dynamic threshold
    img = after_thresholding
    cv2.imshow('5 after thresholding', img)"""

    #Canny edge detector   
    canny_threshold = [100, 500]
    img = cv2.Canny(img, canny_threshold[0], canny_threshold[1]) 
    cv2.imshow('6 after canny', img)

    #Masking
    img = mask_edges(img)
    #cv2.imshow('7 masked', img)    

    # Hough transform
    rho = 6
    theta = 1*np.pi/180
    threshold = 70
    min_line_length = 5   #The minimum number of points that can form a line. Lines with less than this number of points are disregarded.
    max_line_gap = 20      #The maximum gap between two points to be considered in the same line.
    lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), min_line_length, max_line_gap)
    horizontal_midpoint = img.shape[1]/2
    cte = 0 # Cross Track Error
    cnt_left = 0    # number of left lines
    cnt_right = 0   # number of right lines
    
    # Find 'good' lines
    line_image = np.copy(transformed_color_img)*0    
    if lines is None:
        #found nothing
        cv2.waitKey(1)    
        print("nada")
    else:
        left_line_x = []    #x-values of left lines 
        left_line_y = []    #y-values of left lines 
        right_line_x = []   #x-values of right lines 
        right_line_y = []   #x-values of right lines

        for line in lines:
            for x1, y1, x2, y2 in line:
                if x2 - x1 == 0:
                    continue
                slope = (y2 - y1) / float(x2 - x1)
                #===============================
                # filter out irrelevant lines
                if abs(slope)<0.5:
                    continue
                line_len = np.linalg.norm(np.array((x1, y1)) - np.array((x2, y2)))
                if line_len < 20:
                    continue
                #===============================
                
                #draw lines (only relevant ones)
                cv2.line(line_image, (x1, y1), (x2, y2), LINE_COLOR, 2)
                cv2.circle(line_image, (x1, y1), 5, LINE_COLOR, -1)
                cv2.circle(line_image, (x2, y2), 5, LINE_COLOR, -1)
                
                # dims = img.shape(org_image)
                
                #===============================
                # Group lines into L-group and R-group
                #That is, add the X, Y points to the corresponding list defined above
                #(left_line_x, left_line_y, right_line_x, right_line_y)
                
                if x1 < horizontal_midpoint and x2 < horizontal_midpoint:
                    if slope > 0: continue
                    left_line_x.extend((x1, x2))
                    left_line_y.extend((y1, y2))
                    cnt_left += 1
                elif x1 > horizontal_midpoint and x2 > horizontal_midpoint:
                    if slope < 0: continue
                    right_line_x.extend((x1, x2))
                    right_line_y.extend((y1, y2))
                    cnt_right += 1
                #===============================

        # top and bottom of the image
        MIN_Y = 0
        MAX_Y = line_image.shape[0] 

        left_polyfit = None
        right_polyfit = None
        
        if cnt_left > 0:
            #===============================
            # Find the left lane marking   
            fit = np.polyfit(left_line_y, left_line_x, deg=1)
            poly = np.poly1d(fit)
            left_x_start = int(poly(MAX_Y))
            left_x_end = int(poly(MIN_Y))
            #===============================
            
            cv2.line(line_image, (left_x_start, MAX_Y), (left_x_end, MIN_Y), LANE_COLOR, 5)
            
        if cnt_right > 0:
            #===============================
            # Find the right lane marking
            fit = np.polyfit(right_line_y, right_line_x, deg=1) # TODO: should not reuse fit and poly?
            poly = np.poly1d(fit)
            right_x_start = int(poly(MAX_Y))
            right_x_end = int(poly(MIN_Y))
            #===============================

            cv2.line(line_image, (right_x_start, MAX_Y), (right_x_end, MIN_Y), LANE_COLOR, 5)

        print("line found")
        #cv2.imshow('line_image', line_image)
        if cnt_left>0 and cnt_right>0: #detected lane markings at the both sides
            lane_center = (left_x_end + right_x_end) / 2
            car_center = horizontal_midpoint
            cte = car_center - lane_center

            cv2.line(line_image, ((left_x_start+right_x_start)/2, MAX_Y), ((left_x_end + right_x_end)/2, MIN_Y), LANE_COLOR, 5)
            cv2.line(line_image, (horizontal_midpoint, MAX_Y), (horizontal_midpoint, MIN_Y), (255,255,0), 3)
            
            #Draw lane region
            mask = np.zeros_like(line_image)
            imshape = line_image.shape
            vertices = np.array([[(left_x_start,MAX_Y),(left_x_end, MIN_Y), (right_x_end, MIN_Y), (right_x_start, MAX_Y)]], dtype=np.int32)
            cv2.fillPoly(mask, vertices, LANE_REGION_COLOR)
            #Draw lines + lane image
            combo = cv2.addWeighted(line_image, 0.8, mask, 0.2, 0)
            #Overlay lines & lanes on the warped image
            overlay_on_warped_img = cv2.addWeighted(transformed_color_img, 0.5, combo, 0.5, 0)  
            #cv2.imshow('detected lane', overlay_on_warped_img)
            #Overlay lines & lane on the original image (inverse-transformation of perspective)
            overlay_on_org_img = warpPerspective(combo, inverse=True)            
            combo = cv2.addWeighted(overlay_on_org_img, 0.5, org_image, 0.5, 0)              
            cv2.imshow('final', combo)

        elif cnt_left+cnt_right == 0:
            cte = prev_cte ## Just added this
            cv2.imshow('final', org_image)
            print('cannot find any lane markings')            
        else: 
            cte = prev_cte
            cv2.imshow('final', org_image)
            if cnt_left==0:                
                print('cannot find left lane marking')
            if cnt_right == 0:
                print('cannot find right lane marking')

        cv2.waitKey(1)
    prev_cte = cte
    return (cte, cnt_left, cnt_right)
