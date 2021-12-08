import cv2
import numpy as np
import scipy

from geo import *

FieldWidth = 1050
FieldHeight = 680
FieldScale = 0.0075
CAMERA_INDEX = 1

class VisionProcessor():
    def __init__(self, camera_index = CAMERA_INDEX) -> None:
        # add some settings here
        self.cap = cv2.VideoCapture(camera_index)
        self.image = None
        self.wrapped_image = None
        self.M = None

    def close(self):
        self.cap.release()
        cv2.destroyAllWindows()

    def getImage(self):
        """Get image from camera"""
        if self.cap.isOpened():
            self.image = self.cap.read()
        else:
            raise Exception("Camera Not Found!")
        return self.image

    def getMap(self, update = False):
        if update:
            self.getImage()
            self.M = VisionProcessor.align_field(self.image)
            self.wrapped_image = VisionProcessor.warp(self.image, self.M)
        gmap = GridMap(FieldHeight, FieldWidth, FieldScale, 
                    obs_map=VisionProcessor.obstacles_map(self.wrapped_image, color='pink'))
        
        return gmap

    def getBall(self, update = False):
        if update:
            self.getImage()
            if self.M is not None:
                self.wrapped_image = VisionProcessor.warp(self.image, self.M)
        return VisionProcessor.get_ball_xy(self.image, color='brown')

    def getThymio(self, update = True):
        if update:
            self.getImage()
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
        from sklearn import mixture
        from scipy import linalg

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
    def align_field(image, 
                    color = 'green', # color of the field
                    gray_threshold = [28,243],
                    verbose = False):
        '''Used to align the view of the field into a 2D plane'''

        # -- 1. Extract the quadrilateral field ---
        # Performing Colour Filtering to Distinguish the Field
        result= VisionProcessor.color_filter(image, color)
        # if verbose:
        #     cv2.imshow('color filter',result)
        
        # Finding Contours to detect the area of the field
        # convert to a gray image than a binary image.
        gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(gray,gray_threshold[0],gray_threshold[1],0)
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
        c = cv2.convexHull(c)
        # if verbose:
        #     ch = image.copy()
        #     cv2.drawContours(ch, [c], 0, (0,255,0), cv2.FILLED)
        #     cv2.imshow('convexHull',ch)

        # Using GMM to divide 4 segment of the quadrilateral    
        
        ch = np.zeros_like(image)
        cv2.drawContours(ch, [c], 0, (0,255,0), cv2.FILLED)
        gray = cv2.cvtColor(ch,cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray,50,150,apertureSize = 3)
        convexcontour,_ = cv2.findContours(edges,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        lines = VisionProcessor.divide4(convexcontour[0])
        if verbose:
            ch = image.copy()
            cv2.drawContours(ch, [c], 0, (0,255,0), cv2.FILLED)
            for line in lines:
                cv2.circle(ch, ((int)(line[0]), (int)(line[1])), 3, (0, 0, 255), -1)
            cv2.imshow('convexHull',ch)

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
                    print(x, y)
                    cv2.circle(ch, (x, y), 3, (0, 255, 0), -1)
                    cv2.putText(ch,str(i), (x,y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
                    i += 1
        corners = np.array(corners)
        if verbose:
            cv2.imshow('',ch)

        """
            perimeter = cv2.arcLength(c, True)
            # Using an approximate polygon to get the corners of the rectangle
            approx= cv2.approxPolyDP(c, 0.05 * perimeter, True) 
            #[ Question: How can you be sure that `approx` will be a rectangle but n-polydon?]
            
            corners=np.zeros((4,2),dtype=np.int16)
            
            if (len(approx)==0):
                print("Poor color Filtering of field")
                return None

            #if the contours detected don't form a rectangle then we approximate them using a minimum rectangle area
                # if len(approx)>4:
                #         hull =[]
                #         for c in contours:
                #             if cv2.contourArea(c)>10:
                #                 hull.append(cv2.convexHull(c,False))
                #         rect = cv2.minAreaRect(np.concatenate(hull,axis=0))
                #         box = cv2.boxPoints(rect)
                #         box = np.int0(box)
                #         cv2.drawContours(result,[box],0,(0,0,255),2)
                #         c=box
                #         approx= cv2.approxPolyDP(c, 0.05 * perimeter, True)
                #     return

            # Drawing a circle at every corner of the field and 
            # converting the list of corner points to array
            font = cv2.FONT_HERSHEY_SIMPLEX
            for i in range(4):
                point = approx[i]
                x, y = point[0]
                corners[i,0]=x
                corners[i,1]=y    
                if verbose:
                    cv2.circle(result, (x, y), 3, (0, 255, 0), -1)
                    cv2.putText(result,str(i), (x,y), font, 1, (0, 0, 255), 2, cv2.LINE_AA)
                
            if verbose:
                cv2.imshow('',result)
        """
        
        # -- 2. Projection Matrix --
        """
            # Calculating the pairwise distance between every corner point
            distance = scipy.spatial.distance.pdist(corners,'euclidean')
            # Getting rid of the diagonals as they are the two longest distances
            edges=np.sort(corners)[:4]
            #     edges=np.sort(dist)[:4]
            #Since the edges array is sorted the two smallest values are the width and the others are length
            width = edges[:2]
            length = edges[2:]
            
            #     print(width)
            #     return 0
            #Getting Max length and width
            # if image.shape[0]>image.shape[1]:
            #   maxWidth=max(np.max((width).astype(int)),np.max((length).astype(int)))
            #   maxLength=min(np.max((width).astype(int)),np.max((length).astype(int)))
            # else:
            maxWidth=np.max((width).astype(int))
            maxLength=np.max((length).astype(int))
        """
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
    def detect_box(image, color = "yellow"):
        blurred_image = cv2.GaussianBlur(image,(21,21),cv2.BORDER_DEFAULT)
        #Color filtering in yellow
        result3= VisionProcessor.color_filter (blurred_image,color)
        # #--->
        # plt.imshow(cv2.cvtColor(result3, cv2.COLOR_BGR2RGB))
        # plt.show()
        #Contour DEtection
        gray = cv2.cvtColor(result3, cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(gray,120,243,0)
        contours,hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        #Taking contour with biggest area
        cnt= max(contours, key = cv2.contourArea)
        if (len(cnt)==0):
            print("Robot not detected on field")
            return

        #Forming a rectangle of min area arround them
        rect = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(rect)
        box = np.int0(box)

        
        #Contour drawing
        # cv2.drawContours(resized_image,[box],0,(0,0,255),2)

        #Calculating centroid of rectangle using moments
        M = cv2.moments(box)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])

        # cv2.circle(resized_image,(cx,cy),1,(0,255,0),12)

        return cx,cy


    @staticmethod
    def get_robot_pose (image):

        #Detect the yellow box
        cx_yellow,cy_yellow=VisionProcessor.detect_box(image,'yellow')

        #Detect the red box
        cx_red,cy_red=VisionProcessor.detect_box(image,'blue')

        #Save red box center as robot's xy
        robot_xy=Pos(cx_red,cy_red)

        #Draw orientation Vector using yellow and red box centroids
        cv2.arrowedLine(image,(cx_red,cy_red),(cx_yellow,cy_yellow),(255,0,0),3)

        #Draw Point for Centroid of Robot
        cv2.circle(image,(cx_red,cy_red),1,(0,255,0),12)

        # cv2_imshow(image)

        #Calculating the angle of the robot

        #First calculating the horizontal and vertical displacements
        dy=(cy_yellow-cx_red)
        dx=(cx_yellow-cx_red)

        #Calculating the angle between the two dx dy
        #We return the negative value because the origin of image is on the top left corner but by convention we take it at the bottom
        robot_angle=-np.atan2(dx,dy)
        return State(robot_xy,robot_angle)

    @staticmethod
    def obstacles_map(image, color = 'pink', blur_kernel = (19, 19)):
        blurred_image_red = cv2.GaussianBlur(image,blur_kernel,cv2.BORDER_DEFAULT)
        image_red=VisionProcessor.color_filter(blurred_image_red,color)
        kernel = np.ones((5,5),np.uint8)
        
        map=np.empty_like(image_red)

        map[image_red!=0]=0
        map[image_red==0]=255
        erosion = cv2.erode(map,kernel,iterations = 7)
        
        # print("Map")
        # plt.imshow(cv2.cvtColor(erosion, cv2.COLOR_BGR2RGB))
        # plt.show()
        
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
    img = cv2.imread("pink_obs.jpg")
    try:
        # vp = VisionProcessor()
        # gmap = vp.getMap(update=True)
        # thymio_state = vp.getThymio()
        # ball_pos = vp.getBall()
        # gate_pos = vp.getGate()
        M = VisionProcessor.align_field(img, verbose=True)
        wraped = VisionProcessor.warp(img, M)
        cv2.imshow("", wraped)
        obs = VisionProcessor.obstacles_map(wraped)
        cv2.imshow("obs", obs)
        cv2.waitKey(0)
        ball = VisionProcessor.get_ball_xy(wraped)
        cv2.imshow("ball", ball)
        cv2.waitKey(0)
        thymio = VisionProcessor.get_robot_pose(wraped)
        cv2.imshow("thymio", thymio)

        cv2.waitKey(0)
    finally:
        vp.close()

