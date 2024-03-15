import RPi.GPIO as GPIO
from picamera2 import Picamera2 
import cv2 as cv 
from libcamera import controls
import time
from keras.models import load_model  # TensorFlow is required for Keras to work
from PIL import Image, ImageOps  # Install pillow instead of PIL
import numpy as np
import requests
import threading
import json
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from irobot_create_msgs.action import DriveDistance
from irobot_create_msgs.action import RotateAngle

URL = 'https://api.airtable.com/v0/appZIc83IPKKzrCi8/Table%201'
ACCESS_TOKEN = 'patt39D31Bm3pN8Up.38c78c380ad824ecb4e02536d2b847876030e54f828b95d746ef42307921888b'
HEADERS = {'Authorization': 'Bearer ' + ACCESS_TOKEN}


picam2 = Picamera2()
picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous}) #sets auto focus mode
img_name = 'imagetest.jpg'

# GPIO pins for the ultrasonic sensor
# See wiring diagram in /WiringDiagrams/Ultrasonic.png
GPIO_TRIGGER = 40
GPIO_ECHO = 38

GPIO.setmode(GPIO.BOARD)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

# Disable scientific notation for clarity
np.set_printoptions(suppress=True)
model = load_model("keras_model.h5", compile=False)
class_names = open("labels.txt", "r").readlines()
data = np.ndarray(shape=(1, 224, 224, 3), dtype=np.float32)
size = (224, 224)


#take picture:
def picture():
    picam2.capture_file(img_name) #take image 

def measure_distance():
    # Set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)

    start_time = time.time()
    stop_time = time.time()

    # Save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        start_time = time.time()

    # Save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        stop_time = time.time()

    # Time difference between start and arrival
    time_elapsed = stop_time - start_time
    print(time_elapsed)

    # Speed of sound in air (343 meters per second) and 100 for conversion to centimeters
    distance_cm = round((time_elapsed * 34300) / 2, 2)

    print(distance_cm)
    
    time.sleep(0.1)

    return distance_cm

def classify():
    picture()
    image = Image.open('imagetest.jpg').convert("RGB")
    image = ImageOps.fit(image, size, Image.Resampling.LANCZOS)
    image_array = np.asarray(image)
    image.close()
    normalized_image_array = (image_array.astype(np.float32) / 127.5) - 1
    data[0] = normalized_image_array

    # Predicts the model
    prediction = model.predict(data)
    index = np.argmax(prediction)
    class_name = class_names[index]
    name  = class_name[2:]
    confidence_score = prediction[0][index]
     # Print prediction and confidence score
    print("Class:", class_name[2:], end="")
    print("Confidence Score:", confidence_score)
    return name

def command(name, myNode):
    speed , value = fetch_commands_from_airtable(name)
    if value == "Left":
        angle = np.pi/2
    else:
        angle = -1 * np.pi/2
    myNode.RotateGoal(angle, speed)
    while (not myNode.done):
        print("waiting for rotate")
        time.sleep(.1)

def forward(myNode, distance, value):
    myNode.MoveGoal(distance, value)
    while (not myNode.done):
        print("waiting for move")
        time.sleep(.1)
    

def fetch_commands_from_airtable(name):
    response = requests.get(url=URL, headers=HEADERS)
    #print(response.json())
    data = response.json().get('records')
    #print(data)
    for command_record in data:
        fields = command_record.get('fields')
        print(fields)  # Print the entire fields dictionary for debugging
        command = fields.get('Object') + '\n'
        #print(bytes(command, 'utf-8'))
        #print(bytes(name, 'utf-8'))
        if name == command:
          print('object found')
          value = float(fields.get('speed'))
          status = fields.get('Left||Right')
          return value, status

def spin_node(self):
        rclpy.spin(self)

class MovementClient(Node):
    def __init__(self):
        super().__init__('MovementNode')
        self.move_action_client = ActionClient(self, DriveDistance, 'drive_distance')
        self.rotate_action_client = ActionClient(self, RotateAngle, 'rotate_angle')
        self.currentMoveGoal = None
        self.currentRotateGoal = None
        self.done = True

    def MoveGoal(self, distance, speed):
        #print("1")
        self.done = False
        self.currentMoveGoal = DriveDistance.Goal()
        self.currentMoveGoal.distance = float(distance)
        self.currentMoveGoal.max_translation_speed = float(speed)
       # print("2")
        self.move_action_client.wait_for_server()
        self.move_goal_future = self.move_action_client.send_goal_async(self.currentMoveGoal)
        self.move_goal_future.add_done_callback(self.move_goal_response_callback)


    def move_goal_response_callback(self, future):
       # print("3")
        self.move_goal_handle = future.result()
        if not self.move_goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self.move_get_result_future = self.move_goal_handle.get_result_async()
        self.move_get_result_future.add_done_callback(self.move_get_result_callback)

    def move_get_result_callback(self, future):
       # print("4")
        result = future.result()
       # self.get_logger().info(f'Result: {result}')
        self.done = True

    def RotateGoal(self, angle, max_rotation_speed):
        print("rotateGoal")
        self.done = False
        self.currentRotateGoal = RotateAngle.Goal()
        self.currentRotateGoal.angle = float(angle)
        self.currentRotateGoal.max_rotation_speed = float(max_rotation_speed)

        self.rotate_action_client.wait_for_server()
        self.rotate_goal_future = self.rotate_action_client.send_goal_async(self.currentRotateGoal)
        self.rotate_goal_future.add_done_callback(self.rotate_goal_response_callback)

    def rotate_goal_response_callback(self, future):
        self.rotate_goal_handle = future.result()
        if not self.rotate_goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self.rotate_get_result_future = self.rotate_goal_handle.get_result_async()
        self.rotate_get_result_future.add_done_callback(self.rotate_get_result_callback)

    def rotate_get_result_callback(self, future):
        result = future.result()
       # self.get_logger().info(f'Result: {result}')
        self.done = True


def main(args=None):
    try:
        rclpy.init(args=args)
        myNode = MovementClient()
        d = .0127
        speed = .5
        picam2.start() #must start the camera before taking any images
        spin_thread = threading.Thread(target=lambda: rclpy.spin(myNode))
        spin_thread.start()
        step = 0
        while True:
            print("forwards")
            forward(myNode, d, speed)
            step = step + 1
            distance = measure_distance()
            if (distance < 16.5):
                print("close to something")
                name = classify()
                print("Object: " + name)
                command(name, myNode)
            if (step % 15 == 0):
                myNode.RotateGoal(math.radians(0.3),0.3)
                while (not myNode.done):
                    time.sleep(.1)

    except KeyboardInterrupt:
        GPIO.cleanup()
    finally:
        picam2.stop()
        myNode.destroy_node()
        rclpy.shutdown()
        exit()

if __name__ == '__main__':
    main()
