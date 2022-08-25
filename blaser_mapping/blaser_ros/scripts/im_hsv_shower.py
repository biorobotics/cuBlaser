#!/usr/bin/env python
import cv2
import sys
from matplotlib import pyplot as plt

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("usage: python3 im_hsv_shower.py image_file")
        exit(0)
    im = cv2.imread(sys.argv[1])
    im_blur = cv2.medianBlur(im, 3)
    im_hsv = cv2.cvtColor(im_blur, cv2.COLOR_BGR2HSV)

    f, ax = plt.subplots(1,2)
    ax[0].imshow(im_hsv)
    ax[1].imshow(cv2.cvtColor(im_blur, cv2.COLOR_BGR2RGB))
    plt.show()
