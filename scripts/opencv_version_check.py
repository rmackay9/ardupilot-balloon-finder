#
# opencv_version_check - displays the opencv version on the machine
#
#   Start from command line using 'python opencv_version_check.py'.

import cv2

def main():
    # opencv_version_check - prints opencv version 
    print "OpenCV %s" % cv2.__version__

if __name__ == "__main__":
    main()
