from vehicle import Driver
from controller import Camera,Lidar
from controller import Speaker
import numpy as np 
import cv2
import warnings
from scipy.integrate import quad
from scipy.misc import derivative
import time
import threading
import pickle
warnings.filterwarnings("ignore")

driver = Driver()
timestep = int(driver.getBasicTimeStep())

camera = driver.getDevice("camera")

Camera.enable(camera,timestep)

cam_width = 1080 
cam_height = 720

lane_width = 710

ym_per_pix = 30 / cam_height
xm_per_pix = 3.7 / cam_width


speed = 3.5
Kp = 0.218
Ki = 0.11
Kd = -0.003

global left_lfollow
global right_lfollow
global frame,direction_output,dt_traffic
global before_deviation,counter,parking,park_dev
result = np.zeros((13,1))

with open('dt_traffic_last.pkl', 'rb') as f:
    dt_traffic = pickle.load(f)

def function(x):
    return x

def pid_controller(error,fin_time,start_time):
    P = -Kp * error
    (integral_value,_) = quad(lambda x: function(error),start_time,fin_time)
    I = -Ki * integral_value
    derivative_value = derivative(function,error,dx = (fin_time-start_time))
    D = -Kd * derivative_value 
    a = P + I + D
    return a
    
def perspectiveWarp(image):
    global cam_width,cam_height
    
    lane_points = np.float32([[200,570],[880,570],[5,700],[1075,700]])
    transform_points = np.float32([[0,0],[cam_width,0],[0,cam_height],[cam_width,cam_height]])

    matrix = cv2.getPerspectiveTransform(lane_points,transform_points)
    inverse_matrix = cv2.getPerspectiveTransform(transform_points,lane_points)
    
    result = cv2.warpPerspective(image,matrix,(cam_width,cam_height))
    
    return result,inverse_matrix

def proccesImage(image,thresh_min=150,thresh_max=255,kernel=(7,7)):
    
    thresh_canny1 = 40
    thresh_canny2 = 60
    hls = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)
    lower_white = np.array([0, 160, 10])
    upper_white = np.array([255, 255, 255])
    mask = cv2.inRange(image, lower_white, upper_white)
    hls_result = cv2.bitwise_and(image, image, mask = mask)

    gray = cv2.cvtColor(hls_result, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray, thresh_min, thresh_max, cv2.THRESH_BINARY)
    blur = cv2.GaussianBlur(thresh,kernel, 0)
    canny = cv2.Canny(blur, thresh_canny1, thresh_canny2)
    result = cv2.add(thresh,canny)
    
    return result

def plot_histogram(image):
    global right_lfollow,left_lfollow
    histogram = np.sum(image[image.shape[0] // 2:, :], axis = 0)

    midpoint = np.int(histogram.shape[0] / 2)
    return histogram

def right_lane_follow(img,midpoint,histogram):
    global left_lfollow,right_lfollow,before_deviation
    global right_flag
    try:
        right_cam_img = img[:,midpoint:]
        
        out_ind = np.transpose(np.nonzero(right_cam_img))
        
        x_coordinate = out_ind[:,0]
        y_coordinate = out_ind[:,1]
        
        y_coordinate = y_coordinate + midpoint
        
        
        right_fit = np.polyfit(x_coordinate,y_coordinate,2)
        ploty = np.linspace(0, img.shape[0]-1, img.shape[0])
        
        right_fitx = right_fit[0] * ploty**2 + right_fit[1] * ploty + right_fit[2]
             
        rgt_x = np.trunc(right_fitx)
        
        lft_x = rgt_x - lane_width
        
        pts_right = np.array([np.transpose(np.vstack([rgt_x, ploty]))])
        pts_left = np.array([np.flipud(np.transpose(np.vstack([lft_x, ploty])))])
        pts = np.hstack((pts_left, pts_right))
        
        mean_x = np.mean((lft_x, rgt_x), axis=0)
        pts_mean = np.array([np.flipud(np.transpose(np.vstack([mean_x, ploty])))])
        
        mpts = pts_mean[-1][-1][-2].astype(int)
        pixelDeviation = img.shape[1] / 2 - abs(mpts)
        deviation = pixelDeviation * xm_per_pix
        
        print("Right Lane Following : {}".format(deviation))
        before_deviation = deviation
        return deviation
    except:
        deviation = abs(before_deviation) * -1.25
        before_deviation = deviation
        print("Right Lane Disappeared : {}".format(deviation))
        return deviation       

def left_lane_follow(img,midpoint,histogram):
    global left_lfollow,right_lfollow,before_deviation,counter
    global left_flag

    try:
        left_cam_img = img[:,:midpoint]
        
        out_ind = np.transpose(np.nonzero(left_cam_img))
        x_coordinate = out_ind[:,0]
        y_coordinate = out_ind[:,1]

        left_fit = np.polyfit(x_coordinate,y_coordinate,2)
        ploty = np.linspace(0, img.shape[0]-1, img.shape[0])
        
        left_fitx = left_fit[0] * ploty**2 + left_fit[1] * ploty + left_fit[2]
        
        lft_x = np.trunc(left_fitx)
        rgt_x = lft_x + lane_width
    
        pts_left = np.array([np.transpose(np.vstack([lft_x, ploty]))])
        pts_right = np.array([np.flipud(np.transpose(np.vstack([rgt_x, ploty])))])
        pts = np.hstack((pts_left, pts_right))
        
        mean_x = np.mean((lft_x, rgt_x), axis=0)
        pts_mean = np.array([np.flipud(np.transpose(np.vstack([mean_x, ploty])))])
        
        mpts = pts_mean[-1][-1][-2].astype(int)
        pixelDeviation = img.shape[1] / 2 - abs(mpts)
        deviation = pixelDeviation * xm_per_pix
        
        print("Left Lane Following : {}".format(deviation))
        before_deviation = deviation
        return deviation
    except:
        deviation = abs(before_deviation) * 1.25
        before_deviation = deviation
        print("Left Lane Disappeared : {}".format(deviation))
        return deviation


def lane_detection(frame):
    global left_lfollow,right_lfollow
    global left_flag,right_flag,direction_output
    devitation = 0 
    img,interval_matrix = perspectiveWarp(frame)
    img = proccesImage(img)
    histogram = plot_histogram(img)
    midpoint = np.int(histogram.shape[0] / 2)
    
    if direction_output == 1:
        left_lfollow = True
        right_lfollow = False
    elif direction_output == 2:
        left_lfollow = False
        right_lfollow = True
    else :
        pass
    
    if left_lfollow == True and right_lfollow == False:
        deviation = left_lane_follow(img,midpoint,histogram)
        return deviation
    elif left_lfollow == False and right_lfollow == True:
        deviation = right_lane_follow(img,midpoint,histogram)
        return deviation
    else : 
        deviation = 0
        print("Unexpected Error")
        return deviation

def traffic_sign_detection() :
    
    global result,frame,park_dev,parking
    threading.Timer(4.5, traffic_sign_detection).start()
    if len(frame) != 0:
        frame_blob = cv2.dnn.blobFromImage(frame,1/255,(416,416),swapRB = True, crop = False  )
        
        labels=['dur','durak','gecit_yok','ileri_saga','ileri_sola','kirmizi_isik','park','park_yasak','saga_donus_yok','saga_yon','sola_donus_yok','sola_yon','yesil_isik' ]
        
        dt_labels = ['saga_yon','sola_yon','ileri_saga','ileri_sola','sola_donus_yok','saga_donus_yok','gecit_yok']
        
        stop_labels = ["durak","kirmizi_isik","yesil_isik","dur"]        
        
        model = cv2.dnn.readNetFromDarknet("./your-yolov4-obj.cfg", "./your-yolov4_best.weights")
        
        layers = model.getLayerNames()
        
        output_layer = [layers[layer-1]for layer in model.getUnconnectedOutLayers()]
        
        model.setInput(frame_blob)
        detection_layers = model.forward(output_layer)
       
        ids_list = []
        all_sign = []
        for detection_layer in detection_layers:
            for object_detection in detection_layer:
                
                scores = object_detection[5:]
                predicted_id = np.argmax(scores)
                confidence = scores[predicted_id] 
        
                if confidence > 0.70:
                    if not parking : 
                        label = labels[predicted_id]
                        if label == "dur":
                            parking = True
                        if label in stop_labels :
                            print("Stop Labels =>",label)
                        all_sign.append(label)
                        if label in dt_labels:
                            index = dt_labels.index(label)
                            ids_list.append(index)
                    else:
                        label = labels[predicted_id]
                        if label == "park":    
                            bounding_box = object_detection[0:4] * np.array([cam_width, cam_height, cam_width, cam_height])
                            (box_center_x, box_center_y, box_width, box_height) =  bounding_box.astype("int")
                            if box_width > 100 or box_height > 100 :
                                print("Mission Completed")
                            park_dev = box_center_x - (cam_width / 2)
        
        ids_list = np.unique(ids_list)
        all_sign = np.unique(all_sign)
        output = np.zeros(7)
        for i in ids_list:
            print(dt_labels[i])
            output[i] = 1
        print(output)
        dt_direction(output)
     

def parking_algorithm():
    print("Looking for Park Spot : ",park_dev)
    return park_dev / (cam_width/2.7) * -1

def dt_direction(array):
     global direction_output
     direction_output = dt_traffic.predict([array])
     
def initialize():
    global left_lfollow,right_lfollow,before_deviation
    global direction_output,frame,counter
    global left_flag,right_flag,parking,park_dev
    left_lfollow = False 
    right_lfollow = True
    parking = False
    park_dev = 0
    direction_output = 0
    before_deviation = 0
    counter = 0
    frame = []
    traffic_sign_detection()

initialize()

while driver.step() != -1:
    global left_lfollow,right_lfollow
    global frame,direction_output
    
    driver.setCruisingSpeed(speed)
    
    start_time = time.time()
     
    camera_array = camera.getImageArray()
    camera_array = np.array(camera_array,np.uint8)
    camera_array = camera_array.transpose(1,0,2)
    camera_array = cv2.cvtColor(camera_array, cv2.COLOR_BGR2RGB)
    frame = camera_array
 
    
    if not parking :
        dev = lane_detection(camera_array)
        fin_time = time.time() 
        rotation_angle = pid_controller(dev,fin_time,start_time)  
        driver.setSteeringAngle(rotation_angle)
    else:
        dev = parking_algorithm()
        fin_time = time.time() 
        rotation_angle = pid_controller(dev,fin_time,start_time)  
        driver.setSteeringAngle(rotation_angle)
    
