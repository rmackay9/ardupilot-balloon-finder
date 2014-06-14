"""
BalloonVideoBackground : handles pulling an image from the camera in the background and making it easily available to a caller
"""
import sys
import time
from multiprocessing import Process, Pipe
import cv2
from balloon_video import balloon_video

class BalloonVideoBackground:

    # constructor
    def __init__(self):
        # initialise process object
        self.p = None

        # initialise parent pipe
        self.parent_conn = None

        # initialise num images requested
        self.counter = 0

    # image_capture_background - captures all images from the camera in the background and returning the latest image via the pipe when the parent requests it
    def image_capture(self, imgcap_connection, balloon_vid):
        # exit immediately if imgcap_connection is invalid
        if imgcap_connection is None:
            print "image_capture failed because pipe is uninitialised"
            return

        # open the camera
        camera = balloon_vid.get_camera()

        # initialise variables for calculating FPS
        num_images_captured = 0
        num_images_sent = 0
        start_time = time.time()

        # clear latest image
        latest_image = None

        while True:
            # constantly get the image from the webcam
            success_flag, image=camera.read()

            # if successful overwrite our latest image
            if success_flag:
                latest_image = image
                num_images_captured = num_images_captured + 1

            # check if the parent wants the image
            if imgcap_connection.poll():
                recv_obj = imgcap_connection.recv()
                # debug
                print "request received %s" % recv_obj
                # if -1 is received we exit
                if recv_obj == -1:
                    # debug output
                    print "Num Images: %d  FPS rec:%f  FPS sent:%f" % (num_images_captured, num_images_captured/(time.time() - start_time), num_images_sent/(time.time() - start_time))
                    break

                # return image
                imgcap_connection.send(latest_image)
                num_images_sent = num_images_sent + 1

        # release camera
        camera.release()

    # get_image - returns latest image from the camera
    def get_image(self):
        # return immediately if pipe is not initialised
        if self.parent_conn == None:
            return None

        # send request to image capture for image
        self.parent_conn.send(self.counter)

        # increment counter for next time
        self.counter = self.counter + 1

        # wait endlessly until image is returned
        recv_img = self.parent_conn.recv()

        # return image to caller
        return recv_img

    # run - starts background image capture
    def run(self, balloon_vid):
        # create pipe
        self.parent_conn, imgcap_conn = Pipe()

        # create and start the sub process and pass it it's end of the pipe
        self.p = Process(target=self.image_capture, args=(imgcap_conn, balloon_vid))
        self.p.start()

    def stop(self):
        # send exit command to image capture process
        self.parent_conn.send(-1)

        # join process
        self.p.join()

    # main - tests BalloonVideoBackground class
    def main(self):

        # start background process
        self.run(balloon_video)
    
        while True:
            # send request to image capture for image
            img = self.get_image()
    
            # check image is valid
            if not img is None:
                # display image
                cv2.imshow ('image_display', img)
            else:
                print "no image"
    
            # check for ESC key being pressed
            k = cv2.waitKey(5) & 0xFF
            if k == 27:
                break
    
            # take a rest for a bit
            time.sleep(0.1)
    
        # send exit command to image capture process
        self.stop()

# create a single global object
balloon_vid_back = BalloonVideoBackground()

# run main as test
if __name__ == '__main__':
    balloon_vid_back.main()