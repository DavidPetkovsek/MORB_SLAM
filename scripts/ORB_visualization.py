import cv2
import numpy as np

image_query = cv2.imread("scripts/test_image.png", cv2.IMREAD_GRAYSCALE)

orb = cv2.ORB_create(nfeatures=1250)
kp_img, des_img = orb.detectAndCompute(image_query, None)

result_image_query = cv2.drawKeypoints(image_query, kp_img, None, flags=0)
cv2.imshow("image", result_image_query)
cv2.waitKey(0)