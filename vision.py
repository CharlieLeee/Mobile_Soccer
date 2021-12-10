import cv2
import numpy as np
from numpy.lib.type_check import imag
from sklearn import mixture
from scipy import linalg

from geo import *

FieldWidth = 1050
FieldHeight = 680
FieldScale = 0.0075
CAMERA_INDEX = 0

class VisionProcessor():
    def __init__(self, camera_index = CAMERA_INDEX) -> None:
        # add some settings here
        self.cap = None
        self.camera_index = camera_index
        self.image = None
        self.wrapped_image = None
        self.M = None

    def open(self, width = 1280, height = 720):
        self.cap = cv2.VideoCapture(self.camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    def close(self):
        self.cap.release()
        cv2.destroyAllWindows()

    def _getImage(self):
        """Get image from camera"""
        if self.cap.isOpened():
            ret, self.image = self.cap.read()
            if not ret:
                raise Exception("Camera read failed")
        else:
            raise Exception("Camera Not Found!")
        return self.image

    def getMap(self, update = False):
        om = self._getObs(update)
        while om is None:
            om = self._getObs(True)
        return GridMap(FieldHeight, FieldWidth, FieldScale, om)

    def _getObs(self, update = False):
        if update:
            self._getImage()
            self.M = VisionProcessor.align_field(self.image)
            self.wrapped_image = VisionProcessor.warp(self.image, self.M)
        
        return VisionProcessor.obstacles_map(self.wrapped_image, color='pink')
    
    def getBall(self, update = False):
        ballpos = self._getBall(update)
        while ballpos is None:
            ballpos = self._getBall(True)
        return ballpos

    def _getBall(self, update = False):
        if update:
            self._getImage()
            if self.M is not None:
                self.wrapped_image = VisionProcessor.warp(self.image, self.M)
        return VisionProcessor.get_ball_xy(self.image, color='brown')

    def _getThymio(self, update = True):
        if update:
            self._getImage()
            if self.M is not None:
                self.wrapped_image = VisionProcessor.warp(self.image, self.M)
        return VisionProcessor.get_robot_pose(self.image)

    def getGate(self, update = False):
        return Pos(FieldHeight/2, FieldWidth)

    @staticmethod
    def color_filter(image, color):
        '''
        Function used to performed color filtering
        image: The input image on which color filtering is performed
        color: The name of the color which we want to filter
        '''
        #Get a copy of the input image
        result = image.copy()

        #Convert image in HSV color space
        result = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        #Form a mask to filter Red in HSV space
        if color=="red":
            lower1 = np.array([0, 30, 0])
            upper1 = np.array([8, 255,205])
            lower2 = np.array([170,30,0])
            upper2 = np.array([179,255,205])
            lower_mask = cv2.inRange(result, lower1, upper1)
            upper_mask = cv2.inRange(result, lower2, upper2)
            mask = lower_mask + upper_mask;

        #Form a mask to filter green in HSV space
        elif color=="green":
            lower1 = np.array([75, 30, 20])
            upper1 = np.array([90, 225, 225])
            mask = cv2.inRange(result, lower1, upper1) 

        #Form a mask to filter Yellow in HSV space
        elif color=="yellow":
            lower1 = np.array([10, 0, 0])
            upper1 = np.array([32, 255, 255])
            mask = cv2.inRange(result, lower1, upper1) 

        #Form a mask to filter Blue in HSV space
        elif color=="blue":
            lower1 = np.array([110, 120, 0])
            upper1 = np.array([135, 255, 255])
            mask = cv2.inRange(result, lower1, upper1) 

        elif color=="white":
            # sensitivity=45
            lower1 = np.array([0,0,175])
            upper1 = np.array([255,255,185])
            mask = cv2.inRange(result, lower1, upper1) 

        elif color=="black":
            # sensitivity=45
            lower1 = np.array([0,0,0])
            upper1 = np.array([100,100,200])
            mask = cv2.inRange(result, lower1, upper1) 

        elif color=="pink":
            # sensitivity=45
            lower1 = np.array([145,110,110])
            upper1 = np.array([255,255,255])
            mask = cv2.inRange(result, lower1, upper1) 

        else:
            print("Error Color Provided Not Valid")
            assert(False)

        #Apply the mask using a bitwise and operator
        result = cv2.bitwise_and(result, result, mask=mask)
        # cv2_imshow(result)
        return result

    @staticmethod
    def get_ball_xy(image,      # input image to detect the circle on
                    color = "green", 
                    minDist = 750,    # Minimum distance between the centers of the detected circles. 
                    param1 = 12,     # Method uses canny filter edge detection.  
                    param2 = 18,     # accumulator threshold for the circle centers at the detection stage
                    minRadius = 20,  # min radius of circle to detect 
                    maxRadius = 35,   # max radius of circle to detect
                    verbose = False
                    ):
        '''
        image = input image to detect the circle on
        minDist = Minimum distance between the centers of the detected circles. 
                  if too large circles may be missed, 
                  if too small too many circles may appear.
        param1= Method uses canny filter edge detection.  
                This is the higher threshold of the two passed to the Canny edge detector 
                (the lower one is twice smaller of the larger one).
        param2 = accumulator threshold for the circle centers at the detection stage. 
                 The smaller it is, the more false circles may be detected.
        minRadius = min radius of circle to detect 
        maxRadius = max radius of circle to detect
        
        Code based on: https://stackoverflow.com/questions/60637120/detect-circles-in-opencv
        '''
        # Color Filter tha given image        
        filtered_img= VisionProcessor.color_filter(image,color)

        # Converting the image into grayscale
        gray = cv2.cvtColor(filtered_img, cv2.COLOR_BGR2GRAY)

        # Smoothening the Image
        blurred = cv2.medianBlur(gray, 25) 

        # Finds circles in a grayscale image using the Hough transform
        circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, 1, minDist, param1=param1, param2=param2, minRadius=minRadius, maxRadius=maxRadius)

        # If we detected circles with given params
        if circles is not None:
            circles = np.uint16(np.around(circles))

            #Loop through them and draw them on the given image + print coordinates
            for c in circles[0,:]:
                # [ Question: So Ball is the last circle? ]
                x_ball=c[0]
                y_ball=c[1]
                if verbose:
                    cv2.circle(image, (c[0], c[1]), c[2], (0, 255, 0), 2)
                    print("x= ",x_ball,"y= ",y_ball)
                    cv2.circle(image, (x_ball,y_ball), radius=3, color=(0, 0, 255), thickness=-1)
            return x_ball, y_ball
        else:
            if verbose:
                print("Ball Not Found")
            return None
        
    @staticmethod
    def resize_image(img, percent, method = cv2.INTER_AREA):
        '''
        img = input image to be resized
        percent = percentage by which to resize the image
        method = interpolation method. default: INTER_AREA 
        '''
        # percent of original size
        width = int(img.shape[1] * percent / 100)
        height = int(img.shape[0] * percent / 100)
        dim = (width, height)

        # resize image
        resized = cv2.resize(img, dim, interpolation = method)
        return resized

    @staticmethod
    def divide4(contour) -> list:      
        contour = contour.reshape((-1,2))  

        gmm = mixture.GaussianMixture(
                    n_components=4, covariance_type="full"
                )
        gmm.fit(contour)
        mean = gmm.means_
        ret = mean.tolist()
        cov = gmm.covariances_
        for i in range(4):
            v, w = linalg.eigh(cov[i])
            angle = np.arctan2(w[0][1], w[0][0])
            ret[i].append(angle)

        return ret # [mean.x, mean.y, angle]

    @staticmethod
    def intersection(x1, y1, alpha1, x2, y2, alpha2):
        t1 = (y2 - y1 + (x1 - x2)/np.tan(alpha2))/(1/np.tan(alpha1) - 1/np.tan(alpha2))
        x = x1 + t1
        y = y1 + t1/np.tan(alpha1)
        return (int)(x), (int)(y)
        
    @staticmethod
    def corners_gmm(image, 
                    color = 'green', # color of the field
                    #gray_threshold = [28,243],
                    verbose = False):
        """Return corners using GMM

        Args:
            
            color (str, optional): [description]. Defaults to 'green'.
            verbose (bool, optional): [description]. Defaults to False.

        Returns:
            [np.arrays]: corners
        """
        # -- 1. Extract the quadrilateral field ---
        # Performing Colour Filtering to Distinguish the Field
        result= VisionProcessor.color_filter(image, color)
        # if verbose:
        #     cv2.imshow('color filter',result)
        
        # Finding Contours to detect the area of the field
        # convert to a gray image than a binary image.
        gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
        
        (T, thresh) = cv2.threshold(gray, 0, 255,	cv2.THRESH_OTSU)
        #ret,thresh = cv2.threshold(gray,gray_threshold[0],gray_threshold[1],0)
        # morphology operation against noise
        closing = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, np.ones((10, 10)))
        # extract contours
        contours,hierarchy = cv2.findContours(closing,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if verbose:
            ch = image.copy()
            cv2.drawContours(ch,contours,-1,(255,0,0))
            cv2.imshow('contours',ch)

        # Taking the one with the largest area # serve as the opening operation to some extent
        c = max(contours, key = cv2.contourArea)
        
        # Taking its Convex hull to get an aproximate rectangle
        hull = cv2.convexHull(c)
        # if verbose:
        #     ch = image.copy()
        #     cv2.drawContours(ch, [c], 0, (0,255,0), cv2.FILLED)
        #     cv2.imshow('convexHull',ch)

        # Using GMM to divide 4 segment of the quadrilateral   
        hull_img = np.zeros_like(image)     
        length = len(hull)
        for i in range(len(hull)):
            cv2.line(hull_img, tuple(hull[i][0]), tuple(hull[(i+1)%length][0]), (255,0,0), 2)
        hull_img = cv2.cvtColor(hull_img, cv2.COLOR_RGB2GRAY)
        if verbose:
            cv2.imshow("", hull_img)
            
        # gray = cv2.cvtColor(ch,cv2.COLOR_BGR2GRAY)
        # edges = cv2.Canny(gray,50,150,apertureSize = 3)
        convexcontour,_ = cv2.findContours(hull_img,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #print(np.array(convexcontour[0])[:,0])
        lines = VisionProcessor.divide4(convexcontour[0])
        if verbose:
            ch = image.copy()
            cv2.drawContours(ch, [c], 0, (0,255,0), cv2.FILLED)
            for line in lines:
                cv2.circle(ch, ((int)(line[0]), (int)(line[1])), 3, (0, 0, 255), -1)
            cv2.imshow('convexHull',ch)
            cv2.waitKey(10000)

        ll = lines[np.argmax([abs(l[-1]) for l in lines])]
        lines.remove(ll)
        rl = lines[np.argmax([abs(l[-1]) for l in lines])]
        lines.remove(rl)
        ul = lines[0]
        dl = lines[1]
        corners = []
        i = 1
        ch = image.copy()
        for h in [ul, dl]:
            for v in [ll, rl]:
                x, y = VisionProcessor.intersection(h[0], h[1], -h[2], v[0], v[1], -v[2])
                corners.append([x, y])
                if verbose:
                    print("corner",x, y)
                    cv2.circle(ch, (x, y), 3, (0, 255, 0), -1)
                    cv2.putText(ch,str(i), (x,y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
                    i += 1
        corners = np.array(corners)
        if verbose:
            cv2.imshow('',ch)
        means = np.mean(corners, 0)
        for c in corners:
            if c[0] < means[0]:
                if c[1] < means[1]:
                    lt = c
                else:
                    ld = c
            elif c[1] < means[1]:
                rt = c
            else:
                rd = c
        scorners = [lt, ld, rd, rt]
        print(scorners)
        return scorners
    
    def corners_ar(self, verbose=False):
        """Get centers of corners based on aruco markers

        Args:
            verbose (bool, optional): Show images that are used to find centers. Defaults to False.

        Returns:
            centers of corners
        """
        self.open()

        # time.sleep(2.0)
            
        arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)
        arucoParams = cv2.aruco.DetectorParameters_create()

        center = {}
        while True and (len(center.keys())!=4):
            image = self._getImage()

            corners, ids, rejected = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)
            
            
            if len(corners) > 0:
                ids = ids.flatten()
                for (markerCorner, markerID) in zip(corners, ids):
                    corners = markerCorner.reshape((4, 2))
                    (topLeft, topRight, bottomRight, bottomLeft) = corners

                    # Convert to integer pairs
                    topRight = (int(topRight[0]), int(topRight[1]))
                    bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                    bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                    topLeft = (int(topLeft[0]), int(topLeft[1]))
                    cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
                    cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
                    cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
                    cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
                    
                    cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                    cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                    center[markerID] = [cX, cY]
                    cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
            if verbose:
                cv2.imshow("Image", image)
                cv2.waitKey()


        cv2.destroyAllWindows()
        ret_corners = []
        for i in range(1, 5):
            ret_corners.append(center[i])
        return np.array(ret_corners)

    @staticmethod
    def visualize_aruco(img):
        """Visualize detection in Notebook
        Output:
        Corners, image
        """
        image = img.copy()
        arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)
        arucoParams = cv2.aruco.DetectorParameters_create()
        center = {}
        
        corners, ids, _ = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)
        
        
        if len(corners) > 0:
            ids = ids.flatten()
            for (markerCorner, markerID) in zip(corners, ids):
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners

                # Convert to integer pairs
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
                
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.putText(image, str([cX, cY]),(cX, cY), cv2.FONT_HERSHEY_SIMPLEX,\
				0.5, (0, 255, 0), 2)
                center[markerID] = [cX, cY]
                cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
        ret_corners = []
        for i in center.keys():
            ret_corners.append(center[i])
        return np.array(ret_corners), image

    @staticmethod
    def align_field(corners):
        '''Used to align the view of the field into a 2D plane'''
        # sort the corners
        means = np.mean(corners, 0)
        for c in corners:
            if c[0] < means[0]:
                if c[1] < means[1]:
                    lt = c
                else:
                    ld = c
            elif c[1] < means[1]:
                rt = c
            else:
                rd = c
        scorners = [lt, ld, rd, rt]
        print(scorners)
        # Converting to float the distorted rectangle
        input_pts=np.float32(scorners)
        #Building a matrix with the ideal rectangle points (i.e. max distances)
        output_pts = np.float32([[0, 0],
                            [0, FieldHeight - 1],
                            [FieldWidth - 1, FieldHeight - 1],
                            [FieldWidth - 1, 0]])

        # Compute the perspective transform M
        M = cv2.getPerspectiveTransform(input_pts,output_pts)

        #Wrapping the image based on the perspective transform
        return M
        
    @staticmethod
    def warp(image, M, flag=cv2.INTER_LINEAR):
        return cv2.warpPerspective(image, M, (FieldWidth, FieldHeight), flags=flag)

    @staticmethod
    def detect_box(image, color = "yellow", verbose = False):
        blur_kernel = (21, 21)

        blurred_image = cv2.GaussianBlur(image, blur_kernel, cv2.BORDER_DEFAULT)
        #Color filtering in yellow
        result3= VisionProcessor.color_filter (blurred_image,color)
        # #--->
            # plt.imshow(cv2.cvtColor(result3, cv2.COLOR_BGR2RGB))
            # plt.show()
        #Contour DEtection
        gray = cv2.cvtColor(result3, cv2.COLOR_BGR2GRAY)
        (T, thresh) = cv2.threshold(gray, 0, 255,	cv2.THRESH_OTSU)
        opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, np.ones((10,10)))
        if verbose:
            cv2.imshow("", thresh)
            cv2.waitKey(0)
            cv2.imshow("", opening)
            cv2.waitKey(0)
        contours,hierarchy = cv2.findContours(opening, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        #Taking contour with biggest area
        cnt = max(contours, key = cv2.contourArea)
        if (len(cnt)==0):
            print(F"Warning: {color} box not detected on field")
            return None

        #Forming a rectangle of min area arround them
        rect = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(rect)
        box = np.int0(box)


        #Calculating centroid of rectangle using moments
        M = cv2.moments(box)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])

        if verbose:
            ch = image.copy()
            cv2.drawContours(ch, [box],-1,(255,0,0))
            cv2.circle(ch,(cx,cy),1,(0,255,0),12)
            cv2.imshow("", ch)
            cv2.waitKey(0)
        # cv2.circle(resized_image,(cx,cy),1,(0,255,0),12)

        return cx,cy


    @staticmethod
    def get_robot_pose(image, verbose = False):

        #Detect the yellow box
        cx_yellow,cy_yellow=VisionProcessor.detect_box(image,'yellow',verbose)
        #Detect the red box
        cx_red,cy_red=VisionProcessor.detect_box(image,'blue', verbose)
        #Save red box center as robot's xy
        # [TODO] We should set the center of the line connecting two wheels 
        # as the center of the robot
        robot_xy=Pos(cx_red,cy_red)

        if verbose:
            #Draw orientation Vector using yellow and red box centroids
            cv2.arrowedLine(image,(cx_red,cy_red),(cx_yellow,cy_yellow),(255,0,0),3)
            #Draw Point for Centroid of Robot
            cv2.circle(image,(cx_red,cy_red),1,(0,255,0),12)
            cv2.imshow("", image)

        #Calculating the angle of the robot
        #First calculating the horizontal and vertical displacements
        dy=(cy_yellow-cx_red)
        dx=(cx_yellow-cx_red)
        #Calculating the angle between the two dx dy
        # We return the negative value because 
        # the origin of image is on the top left corner 
        # but by convention we take it at the bottom
        robot_angle=-math.atan2(dx,dy)
        return State(robot_xy,robot_angle)

    @staticmethod
    def obstacles_map(image, color = 'pink', blur_kernel = (19, 19), verbose = False):
        blurred_image_red = cv2.GaussianBlur(image,blur_kernel,cv2.BORDER_DEFAULT)
        image_red=VisionProcessor.color_filter(blurred_image_red,color)
        
        map=np.empty_like(image_red)
        map[image_red!=0]=0
        map[image_red==0]=255
        
        if verbose:
            cv2.imshow("",map)
            cv2.waitKey(0)
            kernel = np.ones((5,5),np.uint8)
            erosion = cv2.erode(map,kernel,iterations = 7)        
            cv2.imshow("",erosion)
            cv2.waitKey(0)

        return map

    # --- Visualization ---
    @staticmethod
    def draw_origin(image):
        '''
        Used to draw the origin on the image's left corner
        '''
        im_size=np.shape(image)
        font = cv2.FONT_HERSHEY_SIMPLEX

        #Drawing x-axis
        cv2.arrowedLine(image, (5, im_size[0]-10), (45, im_size[0]-10), (0,0, 255), thickness=2)
        cv2.putText(image, 'x', (45+15, im_size[0]-10), font, 1, (0, 0, 255), 2, cv2.LINE_AA)

        #Drawing y-axis
        cv2.arrowedLine(image, (5, im_size[0]-10), (5, -35+im_size[0]-10), (0,0, 255), thickness=2)
        cv2.putText(image, 'y', (5+15, -35+im_size[0]-10), font, 1, (0, 0, 255), 2, cv2.LINE_AA)

    


if __name__ == "__main__":
    
    # img = cv2.imread("test.jpg")
    # img = cv2.resize(img, (1280, 720))
    # print(img.shape)
    vp = VisionProcessor()
    corners = vp.corners_ar(0)
    print(corners)
    # # corners = VisionProcessor.corners_gmm(img)
    M = VisionProcessor.align_field(corners)
    while True:
        img = vp._getImage()
        warped = vp.warp(img, M)
        cv2.imshow('warped', warped)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
        
    # print(VisionProcessor.detect_box(wraped, color="blue", verbose= True))
    # print(VisionProcessor.detect_box(wraped, color="yellow", verbose= True))
    # vp = VisionProcessor()
    #vp.open()
    # print("camera opened.")
    # img = cv2.imread("img/test.jpg")
    # corners = VisionProcessor.corners_ar(1)
    # corners = np.array([[188,  75], [102, 683],[1159,  682],[1073,   73]])
    # corners, image = VisionProcessor.visualize_aruco(img)
    # M = VisionProcessor.align_field(corners)
    #img = vp._getImage()
    #cv2.imwrite("test.jpg", img)
    # wraped = VisionProcessor.warp(img, M)
    # print(VisionProcessor.detect_box(wraped, color="blue", verbose= True))
    # print(VisionProcessor.detect_box(wraped, color="yellow", verbose= True))
    # print(VisionProcessor.get_robot_pose(warped, verbose=True))
    cv2.waitKey(0)
    # VisionProcessor.obstacles_map(wraped, verbose=True)
    
    # try:
    #     # vp = VisionProcessor()
    #     # gmap = vp.getMap(update=True)
    #     # thymio_state = vp.getThymio()
    #     # ball_pos = vp.getBall()
    #     # gate_pos = vp.getGate()
    #     M = VisionProcessor.align_field(img, verbose=True)
    #     wraped = VisionProcessor.warp(img, M)
    #     cv2.imshow("", wraped)
    #     obs = VisionProcessor.obstacles_map(wraped)
    #     cv2.imshow("obs", obs)
    #     cv2.waitKey(0)
    #     ball = VisionProcessor.get_ball_xy(wraped)
    #     cv2.imshow("ball", ball)
    #     cv2.waitKey(0)
    #     thymio = VisionProcessor.get_robot_pose(wraped)
    #     cv2.imshow("thymio", thymio)

    #     cv2.waitKey(0)
    # finally:
    #     vp.close()

