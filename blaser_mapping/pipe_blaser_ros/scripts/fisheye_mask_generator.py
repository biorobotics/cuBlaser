import cv2
import sys

if len(sys.argv) != 2:
    print("Usage: python fisheye_mask_generator.py <original image>")
    exit(0)

im = cv2.imread(sys.argv[1], cv2.IMREAD_GRAYSCALE)
for x in range(len(im)):
    for y in range(len(im[0])):
        if im[x][y] >= 250:
            im[x][y] = 0
        else:
            im[x][y] = 255
cv2.imwrite("mask_file.png", im)
print("Mask generated at mask_file.png")