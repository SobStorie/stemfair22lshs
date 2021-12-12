 # import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
from collections import deque
import time
import cv2
import argparse
import imutils
import numpy as np
import multiprocessing



#QR CODE Booleans

FollowingQRCodeisFound = False #old multiprocessing boolean
BlockPickupQRCodeisFound = False #old multiprocessing boolean

#Running booleans - track if proccesses are running or not and control delays

followingRunning = False #old multiprocessing boolean
pickupRunning = False # old multiproccessing boolean

runningRed = False # red color following, boolean for if such is runnning
runningBlue = False # blue block, boolean for if such is running
debounce = False #defines whether or not qr code can or should continue
debounceDelay = 2 #how many seconds until debounce becomes false/ qr code scanning works again
timePress = 0 #Time of a successful qr scan. Becomes time.time() upon a scan.

ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
	help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64,
	help="max buffer size")
args = vars(ap.parse_args())

def toggleQRCodeBoolean(qrBoolean):
  if qrBoolean == True:
    qrBoolean = False
  elif qrBoolean == False:
    qrBoolean = True
  else:
    print("WTF? (Line 36)")



def QRCodeDetection():
  global debounce, runningRed, runningBlue, timePress
  #Boolean controller, mostly.
  detector = cv2.QRCodeDetector()
  joe = image.copy()
  # get bounding box coords and data
  data, bbox, joe = detector.detectAndDecode(image)
  
  # if there is a bounding box, draw one, along with the data
  if(bbox is not None):
      for i in range(len(bbox)):
          cv2.line(joe, tuple(bbox[i][0]), tuple(bbox[(i+1) % len(bbox)][0]), color=(255,
                  0, 255), thickness=2)
      cv2.putText(joe, data, (int(bbox[0][0][0]), int(bbox[0][0][1]) - 10), cv2.FONT_HERSHEY_SIMPLEX,
                  0.5, (0, 255, 0), 2)
      if data:
        # Red following will always take priority, coming first. 
        # The code block that repeats could likely be simplified into a function.
          print("data found: ", data)          
          if data == "redColorFollow" and debounce == False:
            if runningRed == False and runningBlue == False:
              #turn on red
              runningBlue = False #failsafe
              timePress = time.time() # Red color following code iw not running
              runningRed = True
              debounce = True
            elif runningBlue == True:
              #Turn off blue, start red
              runningBlue = False
              timePress = time.time() # Red color following code iw not running
              runningRed = True
              debounce = True
            elif runningRed == True:
              #turn of all proccesses
              runningBlue = False
              timePress = time.time() # Red color following code iw not running
              runningRed = False
              debounce = True
            else:
              print("How did you end up here?")
          elif data == "blueBlockPickup"  and debounce == False:
            if runningRed == False and runningBlue == False:
              #turn on red
              runningRed = False #failsafe
              timePress = time.time() # Red color following code iw not running
              runningBlue = True
              debounce = True         
            elif runningRed == True:
              #turn off red, start blue
              runningRed = False
              timePress = time.time() # Red color following code iw not running
              runningBlue = True
              debounce = True
            elif runningBlue == True:
              #Turn off all proccessessss
              runningBlue = False
              timePress = time.time() # Red color following code iw not running
              runningRed = False
              debounce = True

            else:
              print("How did you end up here?")

          else:
            print("An issue occurred, there is probably already a proccess running.")
  # display the image preview
  ##cv2.imshow("code detector", img)
  ## if(cv2.waitKey(1) == ord("q")):


#the following code defines the procedure for following a red laser pointer/circle/ball
def redColorFollow():
  # set the bounds for the red color
    laserColorLower = np.array([136, 87, 111], np.uint8)
    laserColorUpper = np.array([180, 255, 255], np.uint8)

  #create a queue
    pts = deque(maxlen=args["buffer"])

    # resize the frame, blur it, and convert it to the HSV
    # color space   
    imageBlur = imutils.resize(image, width=600)
    blurred = cv2.GaussianBlur(imageBlur, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    # construct a mask for the color "green", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(hsv, laserColorLower, laserColorUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None
      # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
      c = max(cnts, key=cv2.contourArea)
      ((x, y), radius) = cv2.minEnclosingCircle(c)
      M = cv2.moments(c)
      center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        # only proceed if the radius meets a minimum size
      if radius > 10:
          # draw the circle and centroid on the frame,
          # then update the list of tracked points
          cv2.circle(image, (int(x), int(y)), int(radius),
            (0, 255, 255), 2)
          cv2.circle(image, center, 5, (0, 0, 255), -1)
      # update the points queue
      pts.appendleft(center)

#The following code defines the pickup proccess of a blue block
def blueBlockPickup():
  time.sleep(2)

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

# allow the camera to warmup
time.sleep(0.2)
  # capture frames from the camera. Everything is done while the camera is running.
if __name__ == "__main__":
  for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):


        # Part of a multiprocessing solution: define each variable as its own proccess, so we can run these concurrently, multiprocessing creates issues so this is left as is for now.

          # redFollow = multiprocessing.Process(target=redColorFollow)
          # bluePickup = multiprocessing.Process(target=blueBlockPickup)
          # qrDetect = multiprocessing.Process(target=QRCodeDetection)

        # grab the raw NumPy array representing the image, then initialize the timestamp
        
        image = frame.array
        # A copy of the frame if the blurring and such creates issues.
        holdOnToOutput = image.copy()


        #I want qr Detect to only run while debounce is false, but the other functions to repeat constantly, given their boolean is true. For this to work, QR Detect must take no additional time,
        # bringing about the time.time() comparison solution you will see below.

        if debounce == False:
          QRCodeDetection()

        #If QR
        if runningRed == True:
          redColorFollow()
        elif runningBlue == True:
          blueBlockPickup()


        if debounce == True:
          if time.time() - timePress >= debounceDelay:
            debounce = False

            #Multiprocessing solution, creates issues so left out for now.
                  # if FollowingQRCodeisFound == True:
                  #   if followingRunning == False:
                  #   #run laser pointer following
                  #     followingRunning = True
                  #     redFollow.start()
                  #     time.Sleep(2)
                  #   elif followingRunning == True:
                  #     followingRunning = False
                  #     redFollow.terminate()
                  #     time.Sleep(2)
                  # else:
                  #   print("Following QR not Found!")

                  # if BlockPickupQRCodeisFound:
                  #   if pickupRunning == False:
                  #   #run laser pointer following
                  #     pickupRunning = True
                  #     bluePickup.start()
                  #     time.Sleep(2)
                  #   elif pickupRunning == True:
                  #     pickupRunning = False
                  #     bluePickup.terminate()
                  #     time.Sleep(2)
                  # else:
                  #   print("Block Pickup QR not Found!")

        # show the frame
        cv2.imshow("Frame", image)
        key = cv2.waitKey(1) & 0xFF
        
        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)

        

        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            break

cv2.destroyAllWindows()