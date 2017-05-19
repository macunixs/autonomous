#!/usr/bin/env python
# coding: Latin-1
#from __future__ import print_function
# Load library functions we want
import time
import os
import sys
import io
import threading
import picamera
import picamera.array
import cv2
import numpy
import ZeroBorg

# Re-direct our output to standard error, we need to ignore standard out to hide some nasty print statements from pygame
sys.stdout = sys.stderr
global ZB

print 'Libraries loaded'
ZB = ZeroBorg.ZeroBorg()
ZB.Init()
if not ZB.foundChip:
    boards = ZeroBorg.ScanForZeroBorg()
    if len(boards) == 0:
        print 'No ZeroBorg found, check you are attached :)'
    else:
        print 'No ZeroBorg at address %02X, but we did find boards:' % (ZB.i2cAddress)
        for board in boards:
            print '    %02X (%d)' % (board, board)
        print 'If you need to change the I�C address change the setup line so it is correct, e.g.'
        print 'ZB.i2cAddress = 0x%02X' % (boards[0])
    sys.exit()
#ZB.SetEpoIgnore(True)                 # Uncomment to disable EPO latch, needed if you do not have a switch / jumper
# Ensure the communications failsafe has been enabled!
failsafe = False
for i in range(5):
    ZB.SetCommsFailsafe(True)
    failsafe = ZB.GetCommsFailsafe()
    if failsafe:
        break
if not failsafe:

    print 'Board %02X failed to report in failsafe mode!' % (ZB.i2cAddress)
    sys.exit()
ZB.ResetEpo()
ZB.MotorsOff()

# Global values

global running
global camera
global processor
global imgcpy
global upper_black
global lower_black

running = True


# Power settings
voltageIn = 7.2                        # Total battery voltage to the PicoBorg Reverse
voltageOut = 6.0                        # Maximum motor voltage

# Camera settings
imageWidth  = 320                       # Camera image width
imageHeight = 240                       # Camera image height
frameRate = 30                          # Camera image capture frame rate

# Auto drive settings
autoMaxPower = 1.0                      # Maximum output in automatic mode
autoMinPower = 0.3                      # Minimum output in automatic mode
autoMinArea = 50                        # Smallest target to move towards
autoMaxArea = 15000                     # Largest target to move towards
autoFullSpeedArea = 10000                 # Target size at which we use the maximum allowed output

# Setup the power limits
##if voltageOut > voltageIn:
##    maxPower = 1.0
##else:
##    maxPower = voltageOut / float(voltageIn)
maxPower = 1.2
autoMaxPower *= maxPower

# Image stream processing thread
class StreamProcessor(threading.Thread):
    def __init__(self):
        super(StreamProcessor, self).__init__()
        self.stream = picamera.array.PiRGBArray(camera)
        self.event = threading.Event()
        self.terminated = False
        self.start()
        self.begin = 0

    def run(self):
        # This method runs in a separate thread
        while not self.terminated:
            # Wait for an image to be written to the stream
            if self.event.wait(1):
                try:
                    ##### Read the image and do some processing on it
                    self.stream.seek(0)
                    self.ProcessImage(self.stream.array)
                finally:
                    ##### Reset the stream and event
                    self.stream.seek(0)
                    self.stream.truncate()
                    self.event.clear()


    # Image processing function
    def ProcessImage(self, image):
        # Get the image
        global imgcpy
        global cropped
        imgcpy = image.copy()
##        print("dimension:{}".format(imgcpy.shape))

##        -------------------------------resizing method if you need it-------------------------------
##        r = 100.0 / imgcpy.shape[1]
##        dim = (100,int(imgcpy.shape[0]*r))
##        resized = cv2.resize(imgcpy,dim,interpolation=cv2.INTER_AREA)
##        cv2.imshow('resized-image',resized)
##        print("resized dimension:{}".format(resized.shape))


##   crop the image from (240,320) to 140:240,0:320
        cropped = imgcpy[160:240 , 0:320]
        #cv2.imshow("cropped",cropped)
        blur = cv2.GaussianBlur(cropped,(5,5),0)

        blur = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV) # image for blue and green HSV

        black = cv2.inRange(blur, lower_black, upper_black)
        cv2.imshow('black',black)
        edgesLine= cv2.Canny(black,50,200,255)
        #cv2.imshow('line edges',edgesLine)

        # Find the contours
        _,contours,hierarchy = cv2.findContours(edgesLine, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #print("contourS:{}".format(contours))
        cv2.drawContours(blur, contours, -1, (255,255,255), 1)
        cv2.imshow('contour-from-edges-black',blur)

        # Go through each contour
        foundArea = -1
        foundX = -1
        foundY = -1
        for contour in contours:
            x,y,w,h = cv2.boundingRect(contour)
            cx = x + (w / 2)
            cy = y + (h / 2)
            area = w * h # to get the area of bounding box of black hsv

            if foundArea < area: # if area is larger than -1, accept that value as area
                foundArea = area
                foundX = cx
                foundY = cy
                if area > autoMinArea:
                     cv2.rectangle(cropped,(x,y),(x+w,y+h),(255,255,0),1)
                     cv2.putText(cropped,'*'+str(cy),(cx,cy), font, 0.5,(0,255,255))

        if foundArea > 0:
            line = [foundX, foundY, foundArea]
        else:
            line = None
        cv2.waitKey(1)

        # Set drives or report ball status
        self.SetSpeedFromLine(line)

    # Set the motor speed from the ball position
    def SetSpeedFromLine(self, line):
        global ZB
        global cropped
        invertRotation = True
        MULTIPLIER = 1
        driveLeft  = 0.0
        driveRight = 0.0
        if line:    # statement is True if line is not None ! means Line is detected
            x = line[0]
            y = line[1]
            cv2.putText(cropped,'x-coor:'+str(x),(x,y-20), font, 0.5,(0,255,255))


            area = line[2]
            print("area:{}".format(area))
            if area < autoMinArea:
                print 'Scanning for target...'
            elif area > autoMaxArea:
                print 'Target close enough! Stop approach!'
            else:
                if area < autoFullSpeedArea: # compare with 500
                    speed = 1.0
                    print("speed set:{}".format(speed))
                else:
                    speed = 1.0 / (area / autoFullSpeedArea)
                    print("speed modify:{}".format(speed))

##                speed *= autoMaxPower - autoMinPower
##                speed += autoMinPower

                direction = (x - imageCentreX) / imageCentreX
                if direction < -0.2:
                    # Turn right
                    driveRight  = speed
                    driveLeft = speed * (1.0 + direction)
                elif direction > 0.2:
                    # Turn left
                    driveRight  = speed * (1.0 - direction)
                    driveLeft = speed
                else:
                    driveRight  = speed * MULTIPLIER
                    driveLeft = speed * MULTIPLIER

                cv2.putText(cropped,'L:'+str(driveLeft),(x-50,y+20), font, 0.4,(0,255,255))
                cv2.putText(cropped,'R:'+str(driveRight),(x-50,y+40), font, 0.4,(0,255,255))
                cv2.imshow('line-detection-cropped',cropped)
                #print("ball x-axis:{:.2f}".format(x))
                print("dir:{:.2f} L:{:.2f} R:{:.2f}".format(direction,driveLeft,driveRight))
        else:
            print 'No line detected' # when Line is not detected Print this out!

        if invertRotation:
            driveLeft = -driveLeft
            driveRight = -driveRight


        ZB.SetMotor1(driveLeft)
        ZB.SetMotor3(driveRight)

# Image capture thread
class ImageCapture(threading.Thread):
    def __init__(self):
        super(ImageCapture, self).__init__()
        self.start()

    def run(self):
        global camera
        global processor
        print 'Start the stream using the video port'
        camera.capture_sequence(self.TriggerStream(), format='bgr', use_video_port=True)
        print 'Terminating camera processing...'
        processor.terminated = True
        processor.join()
        print 'Processing terminated.'

    # Stream delegation loop
    def TriggerStream(self):
        global running
        while running:
            if processor.event.is_set():
                time.sleep(0.01)
            else:
                yield processor.stream
                processor.event.set()

# Startup sequence
print 'Setup camera'
lower_black = numpy.array([0, 0, 0])            # [min_hue, min_sat, min_val]
upper_black = numpy.array([180, 255, 60]) # [max_hue,max_sat,max_val]
camera = picamera.PiCamera()
camera.resolution = (imageWidth, imageHeight)
camera.framerate = frameRate
camera.hflip = True
camera.vflip = True
imageCentreX = imageWidth / 2.0
imageCentreY = imageHeight / 2.0
font = cv2.FONT_HERSHEY_SIMPLEX

print 'Setup the stream processing thread'
print 'Initializing OpenCV capture stream'
print 'Creating window *livestream*'
processor = StreamProcessor()

print 'Wait ...'
time.sleep(2)
captureThread = ImageCapture()

try:


    print 'Press CTRL+C to quit'
    # Loop indefinitely
    while running:
        #print("this is main loop")

        # Change the LED to reflect the status of the EPO latch
        # We do this regularly to keep the communications failsafe test happy

        time.sleep(1)
    # Disable all drives

except KeyboardInterrupt:
    # CTRL+C exit, disable all drives
    ZB.MotorsOff()
    print '\nProcess Interrupted by Ctrl+C'
    print 'Shuting Down program...'

except:
    # Unexpected error, shut down!
    e = sys.exc_info()[0]
    print
    print e
    print '\nUnexpected error, shutting down!'

# Tell each thread to stop, and wait for them to end

running = False
captureThread.join()
processor.terminated = True
processor.join()
del camera
cv2.destroyAllWindows()
ZB.MotorsOff()
print 'Program terminated.'
