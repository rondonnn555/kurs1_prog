import paho.mqtt.client as mqtt
from collections import deque
import mediapipe as mp
import numpy as np
import cv2 as cv
import threading
import struct
import time
import os

import matplotlib.pyplot as plt


Image = mp.Image
ImageFormat = mp.ImageFormat
BaseOptions = mp.tasks.BaseOptions

GestureRecognizer = mp.tasks.vision.GestureRecognizer
GestureRecognizerOptions = mp.tasks.vision.GestureRecognizerOptions
GestureRecognizerResult = mp.tasks.vision.GestureRecognizerResult

HandLandmarker = mp.tasks.vision.HandLandmarker
HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
HandLandmarkerResult = mp.tasks.vision.HandLandmarkerResult

VisionRunningMode = mp.tasks.vision.RunningMode

cv.CAP_PROP_BUFFERSIZE, 3


class FpsCounter():
    def __init__(self, history_length):
        self.frame_time = 0

        self.fps = 0
        self.avg_fps = 0
        self.fps_history = deque(maxlen=history_length)

        self.last_frame = time.perf_counter()

    def update(self):
        t = time.perf_counter()

        self.frame_time = t - self.last_frame
        self.last_frame = t

        self.fps = 1/self.frame_time
        self.fps_history.append(self.fps)
        self.avg_fps = sum(self.fps_history) / len(self.fps_history)

        return self.avg_fps


class VideoProcessor():
    def __init__(self, fps_counter: FpsCounter, num_hands, max_queue, camera_path, model_asset_path):
        base_options = BaseOptions(model_asset_path=model_asset_path)

        options = HandLandmarkerOptions(
            num_hands        = num_hands,
            base_options     = base_options,
            running_mode     = VisionRunningMode.LIVE_STREAM,
            result_callback  = self.result_callback,
        )

        self.processor = HandLandmarker.create_from_options(options)

        self.cap = cv.VideoCapture(camera_path)

        self.queue = 0
        self.queue_size = max_queue
        self.last_image = None
        self.last_result = None
        self.last_timestamp = None

        self.fps = 0
        self.fps_counter = fps_counter
        
        self.capture_thread = threading.Thread(target=self.capture_loop)
        self.capture_thread.start()

    def result_callback(self, result, output_image, timestamp_ms):
        self.fps = self.fps_counter.update()
        np_image = output_image.numpy_view()
        self.last_image = cv.cvtColor(np_image, cv.COLOR_RGB2BGR)
        self.last_timestamp = timestamp_ms
        self.last_result = result
        self.queue -= 1

    def capture_loop(self):
        while True:
            if self.queue < self.queue_size:
                self.queue += 1
                ret, frame = self.cap.read()

                if not ret:
                    print("ret == False, capture ended")
                    exit()

                rgb_frame = cv.cvtColor(frame, cv.COLOR_BGRA2RGB)
                image = Image(image_format=ImageFormat.SRGB, data=rgb_frame)
                self.last_timestamp = round(time.perf_counter() * 1000)
                self.processor.detect_async(image, self.last_timestamp)
            else:
                time.sleep(.001)

    def get_last_result(self):
        return(
            self.last_image,
            self.last_result,
            self.last_timestamp
        )


class Go1Controller():
    def __init__(self, cmd_interval, max_walk_speed, max_yaw_speed, max_tilt):
        robot_ip = "192.168.12.1"
        robot_port = 1883

        self.mqttc = mqtt.Client()
        self.mqttc.on_message = self.on_message
        self.mqttc.on_connect = self.on_connect
        self.mqttc.on_publish = self.on_publish
        self.mqttc.on_subscribe = self.on_subscribe
        self.mqttc.on_log = self.on_log
        self.mqttc.connect(robot_ip, robot_port)
        self.mqttc.loop_start()

        self.stick = [0, 0, 0, 0]

        self.max_walk_speed = max_walk_speed
        self.max_yaw_speed = max_yaw_speed
        self.max_tilt = max_tilt

        self.cmd_interval = cmd_interval

        self.action = False
        self.action_time = 0
        self.action_length = 0
        self.mode = "stand"
        self.pre_action_mode = "stand"

        self.stick_thread = threading.Thread(target=self.stick_loop)
        self.stick_thread.start()
    
    def publish(self, topic, message):
        self.mqttc.publish(
            topic,
            message,
            qos=2
        ).wait_for_publish()

    # modes

    def standUp(self):
        self.mode = "damping"
        topic = "controller/action"
        message = "standUp"
        self.publish(topic, message)

    def standDown(self):
        self.mode = "damping"
        topic = "controller/action"
        message = "standDown"
        self.publish(topic, message)

    def stand(self):
        self.mode = "stand"
        topic = "controller/action"
        message = "stand"
        self.publish(topic, message)

    def walk(self):
        self.mode = "walk"
        topic = "controller/action"
        message = "walk"
        self.publish(topic, message)

    def run(self):
        self.mode = "run"
        topic = "controller/action"
        message = "run"
        self.publish(topic, message)

    def climb(self):
        self.mode = "climb"
        topic = "controller/action"
        message = "climb"
        self.publish(topic, message)

    def set_mode(self, mode):
        if mode == "stand": self.stand()
        elif mode == "walk": self.walk()
        elif mode == "run": self.run()
        elif mode == "climb": self.climb()
        else: print(f"unknown mode: {mode}")

    # actions

    def action(t=None, arg=None):
        # block controls for the duration of the action
        # and restore mode that was acive priot to the action
        def decorator(func):
            def wrapper(self, *args, **kwargs):
                self.action = True
                
                self.set_stick(0, 0, 0, 0)
                self.pre_action_mode = self.mode
                self.action_time = time.perf_counter()
                self.action_length = t if t is not None else kwargs.get(arg)
                return func(self, *args, **kwargs)
            return wrapper
        return decorator
    
    @action(t=6) # pray takes ~6 seconds
    def pray(self):
        topic = "controller/action"
        message = "straightHand1"
        self.publish(topic, message)

    @action(t=10) # TODO measure length
    def dance1(self):
        topic = "controller/action"
        message = "dance1"
        self.publish(topic, message)

    @action(t=10) # TODO measure length
    def dance2(self):
        topic = "controller/action"
        message = "dance2"
        self.publish(topic, message)

    @action(t=10) # TODO measure length
    def dance3(self):
        topic = "controller/action"
        message = "dance3"
        self.publish(topic, message)

    @action(t=10) # TODO measure length
    def dance4(self):
        topic = "controller/action"
        message = "dance4"
        self.publish(topic, message)

    @action(t=10) # TODO measure length
    def jump_yaw(self):
        topic = "controller/action"
        message = "jumpYaw"
        self.publish(topic, message)

    @action(t=10) # TODO measure length
    def backflip(self):
        print("Backflip disabled")
        # topic = "controller/action"
        # message = "backflip"
        # self.publish(topic, message)
    
    @action(arg="time_s")
    def spin(self, speed, time_s=None):
        self.walk()
        self.set_stick(0, 0, speed, 0)

    # control

    def clamp(self, x, low, high):
        return low if x < low else (high if x > high else x)

    def stick_loop(self):
        while True:
            topic = "controller/stick"
            message = struct.pack('ffff', self.stick[0], self.stick[1], self.stick[2], self.stick[3])
            self.publish(topic, message)

            if self.action and (time.perf_counter() - self.action_time > self.action_length):
                self.set_mode(self.pre_action_mode)
                self.action = False
                self.set_stick(0, 0, 0, 0)

            time.sleep(self.cmd_interval)

    def set_stick(self, forward_speed, right_speed, yaw_speed, tilt):
        if self.action:
            return
        
        forward_speed = self.clamp(forward_speed, -self.max_walk_speed, self.max_walk_speed)
        right_speed = self.clamp(right_speed, -self.max_walk_speed, self.max_walk_speed)
        yaw_speed = self.clamp(yaw_speed, -self.max_yaw_speed, self.max_yaw_speed)
        tilt = self.clamp(tilt, -self.max_tilt, self.max_tilt)
        self.stick = [right_speed, yaw_speed, tilt, forward_speed]

    def set_speed(self, forward_speed, right_speed):
        if self.action:
            return
        
        forward_speed = self.clamp(forward_speed, -self.max_walk_speed, self.max_walk_speed)
        right_speed = self.clamp(right_speed, -self.max_walk_speed, self.max_walk_speed)
        self.stick[0] = right_speed
        self.stick[3] = forward_speed

    def set_yaw(self, yaw_speed):
        if self.action:
            return
        
        yaw_speed = self.clamp(yaw_speed, -self.max_yaw_speed, self.max_yaw_speed)
        self.stick[1] = yaw_speed

    def set_tilt(self, tilt):
        if self.action:
            return
        
        tilt = self.clamp(tilt, -self.max_tilt, self.max_tilt)
        self.stick[2] = tilt

    def reset(self):
        self.action = False
        self.set_stick(0, 0, 0, 0)
        self.stand()

    # mqtt debug

    def on_connect(self, mqttc, obj, flags, rc):
        # print("rc: " + str(rc))
        pass


    def on_message(self, mqttc, obj, msg):
        # print(msg.topic + " " + str(msg.qos) + " " + str(msg.payload))
        pass


    def on_publish(self, mqttc, obj, mid):
        # print("mid: " + str(mid))
        pass


    def on_subscribe(self, mqttc, obj, mid, granted_qos):
        # print("Subscribed: " + str(mid) + " " + str(granted_qos))
        pass


    def on_log(self, mqttc, obj, level, string):
        # print(string)
        pass

def get_point_list(image_width, image_height, landmarks):
    landmark_points = []

    for _, landmark in enumerate(landmarks):
        landmark_x = min(int(landmark.x * image_width), image_width - 1)
        landmark_y = min(int(landmark.y * image_height), image_height - 1)

        landmark_points.append([landmark_x, landmark_y])

    return landmark_points

def draw_landmarks(image, landmark_points):
    if len(landmark_points) > 0:
        # Большой палец
        cv.line(image, tuple(landmark_points[2]), tuple(landmark_points[3]), (0, 0, 0), 6)
        cv.line(image, tuple(landmark_points[2]), tuple(landmark_points[3]), (255, 255, 255), 2)
        cv.line(image, tuple(landmark_points[3]), tuple(landmark_points[4]), (0, 0, 0), 6)
        cv.line(image, tuple(landmark_points[3]), tuple(landmark_points[4]), (255, 255, 255), 2)

        # Указательный палец
        cv.line(image, tuple(landmark_points[5]), tuple(landmark_points[6]), (0, 0, 0), 6)
        cv.line(image, tuple(landmark_points[5]), tuple(landmark_points[6]), (255, 255, 255), 2)
        cv.line(image, tuple(landmark_points[6]), tuple(landmark_points[7]), (0, 0, 0), 6)
        cv.line(image, tuple(landmark_points[6]), tuple(landmark_points[7]), (255, 255, 255), 2)
        cv.line(image, tuple(landmark_points[7]), tuple(landmark_points[8]), (0, 0, 0), 6)
        cv.line(image, tuple(landmark_points[7]), tuple(landmark_points[8]), (255, 255, 255), 2)

        # Средний палец
        cv.line(image, tuple(landmark_points[9]), tuple(landmark_points[10]), (0, 0, 0), 6)
        cv.line(image, tuple(landmark_points[9]), tuple(landmark_points[10]), (255, 255, 255), 2)
        cv.line(image, tuple(landmark_points[10]), tuple(landmark_points[11]), (0, 0, 0), 6)
        cv.line(image, tuple(landmark_points[10]), tuple(landmark_points[11]), (255, 255, 255), 2)
        cv.line(image, tuple(landmark_points[11]), tuple(landmark_points[12]), (0, 0, 0), 6)
        cv.line(image, tuple(landmark_points[11]), tuple(landmark_points[12]), (255, 255, 255), 2)

        # Безымянный палец
        cv.line(image, tuple(landmark_points[13]), tuple(landmark_points[14]), (0, 0, 0), 6)
        cv.line(image, tuple(landmark_points[13]), tuple(landmark_points[14]), (255, 255, 255), 2)
        cv.line(image, tuple(landmark_points[14]), tuple(landmark_points[15]), (0, 0, 0), 6)
        cv.line(image, tuple(landmark_points[14]), tuple(landmark_points[15]), (255, 255, 255), 2)
        cv.line(image, tuple(landmark_points[15]), tuple(landmark_points[16]), (0, 0, 0), 6)
        cv.line(image, tuple(landmark_points[15]), tuple(landmark_points[16]), (255, 255, 255), 2)

        # Мизинец
        cv.line(image, tuple(landmark_points[17]), tuple(landmark_points[18]), (0, 0, 0), 6)
        cv.line(image, tuple(landmark_points[17]), tuple(landmark_points[18]), (255, 255, 255), 2)
        cv.line(image, tuple(landmark_points[18]), tuple(landmark_points[19]), (0, 0, 0), 6)
        cv.line(image, tuple(landmark_points[18]), tuple(landmark_points[19]), (255, 255, 255), 2)
        cv.line(image, tuple(landmark_points[19]), tuple(landmark_points[20]), (0, 0, 0), 6)
        cv.line(image, tuple(landmark_points[19]), tuple(landmark_points[20]), (255, 255, 255), 2)

        # Ладонь
        cv.line(image, tuple(landmark_points[0]), tuple(landmark_points[1]), (0, 0, 0), 6)
        cv.line(image, tuple(landmark_points[0]), tuple(landmark_points[1]), (255, 255, 255), 2)
        cv.line(image, tuple(landmark_points[1]), tuple(landmark_points[2]), (0, 0, 0), 6)
        cv.line(image, tuple(landmark_points[1]), tuple(landmark_points[2]), (255, 255, 255), 2)
        cv.line(image, tuple(landmark_points[2]), tuple(landmark_points[5]), (0, 0, 0), 6)
        cv.line(image, tuple(landmark_points[2]), tuple(landmark_points[5]), (255, 255, 255), 2)
        cv.line(image, tuple(landmark_points[5]), tuple(landmark_points[9]), (0, 0, 0), 6)
        cv.line(image, tuple(landmark_points[5]), tuple(landmark_points[9]), (255, 255, 255), 2)
        cv.line(image, tuple(landmark_points[9]), tuple(landmark_points[13]), (0, 0, 0), 6)
        cv.line(image, tuple(landmark_points[9]), tuple(landmark_points[13]), (255, 255, 255), 2)
        cv.line(image, tuple(landmark_points[13]), tuple(landmark_points[17]), (0, 0, 0), 6)
        cv.line(image, tuple(landmark_points[13]), tuple(landmark_points[17]), (255, 255, 255), 2)
        cv.line(image, tuple(landmark_points[17]), tuple(landmark_points[0]), (0, 0, 0), 6)
        cv.line(image, tuple(landmark_points[17]), tuple(landmark_points[0]), (255, 255, 255), 2)

    # Ключевые точки
    for index, landmark in enumerate(landmark_points):
        if index == 0:
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255), -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 1:
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255), -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 2:
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255), -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 3:
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255), -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 4:
            cv.circle(image, (landmark[0], landmark[1]), 8, (255, 255, 255), -1)
            cv.circle(image, (landmark[0], landmark[1]), 8, (0, 0, 0), 1)
        if index == 5:
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255), -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 6:
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255), -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 7:
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255), 1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 8:
            cv.circle(image, (landmark[0], landmark[1]), 8, (255, 255, 255), -1)
            cv.circle(image, (landmark[0], landmark[1]), 8, (0, 0, 0), 1)
        if index == 9:
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255), -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 10:
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255), -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 11:
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255), -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 12:
            cv.circle(image, (landmark[0], landmark[1]), 8, (255, 255, 255), -1)
            cv.circle(image, (landmark[0], landmark[1]), 8, (0, 0, 0), 1)
        if index == 13:
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255), -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 14:
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255), -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 15:
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255), -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 16:
            cv.circle(image, (landmark[0], landmark[1]), 8, (255, 255, 255), -1)
            cv.circle(image, (landmark[0], landmark[1]), 8, (0, 0, 0), 1)
        if index == 17:
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255), -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 18:
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255), -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 19:
            cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255), -1)
            cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
        if index == 20:
            cv.circle(image, (landmark[0], landmark[1]), 8, (255, 255, 255), -1)
            cv.circle(image, (landmark[0], landmark[1]), 8, (0, 0, 0), 1)

    return image

def get_hand_center(hand_landmarks):
    num = len(hand_landmarks)
    sum_x = 0
    sum_y = 0

    for landmark in hand_landmarks:
        sum_x += landmark.x
        sum_y += landmark.y

    return (sum_x/num, sum_y/num)

def cut_feqs(freq, fft):
    _fft = []
    _freq = []
    for r, f in zip(fft, freq):
        if f >= 0:
            _fft.append(r)
            _freq.append(f)
    return _freq, _fft

def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1280,
    capture_height=720,
    display_width=640,
    display_height=360,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "appsink drop=1"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

if __name__ == "__main__":
    fps_counter = FpsCounter(15)
    dog_controller = Go1Controller(
        cmd_interval=0.1,
        max_walk_speed=0.6,
        max_yaw_speed=0.6,
        max_tilt=0.6
    )

    dog_controller.set_stick(0, 0, 0, 0)
    dog_controller.standDown()
    time.sleep(4)
    dog_controller.standUp()
    time.sleep(2)
    dog_controller.stand()
    time.sleep(1)

    # tracking coefficients
    yaw_c = 0.3
    tilt_c = 0.3

    # state
    idle = False
    last_detection = 0
    required_time = 1.7
    current_gesture = None
    gesture_time = time.perf_counter()

    base_options = BaseOptions(model_asset_path="gesture_recognizer.task")
    options = GestureRecognizerOptions(
        num_hands        = 1,
        base_options     = base_options,
        running_mode     = VisionRunningMode.VIDEO,
    )

    processor = GestureRecognizer.create_from_options(options)

    cap = cv.VideoCapture(gstreamer_pipeline())

    # cap = cv.VideoCapture(0)
    while True:
        timestamp_ms = round(time.perf_counter() * 1000)
        ret, image = cap.read()

        if image is None:
            continue

        rgb_image = cv.cvtColor(image, cv.COLOR_BGR2RGB)
        mp_image = Image(image_format=ImageFormat.SRGB, data=rgb_image)
        result = processor.recognize_for_video(mp_image, timestamp_ms)

        #print(fps_counter.update())

        if len(result.hand_landmarks) > 0:
            idle = False
            last_detection = time.perf_counter()

            #points = get_point_list(image.shape[1], image.shape[0], result.hand_landmarks[0])
            #image = draw_landmarks(image, points)

            if not dog_controller.action:
                cx, cy = get_hand_center(result.hand_landmarks[0])
                cx, cy = (cx-0.5)*2, (cy-0.5)*2
                print(f"hand detected: {cx=} {cy=}")

                yaw = dog_controller.stick[1]
                dog_controller.set_yaw(yaw + cx * yaw_c)
                print(f"set yaw to {yaw + cx * yaw_c}")

                tilt = dog_controller.stick[2]
                dog_controller.set_tilt(tilt + cy * tilt_c)
                print(f"set yatiltw to {tilt + cy * tilt_c}")

        if ((not idle) and time.perf_counter() - last_detection > 4):
            idle = True
            dog_controller.set_tilt(-0.8)
            dog_controller.set_yaw(0)

        if len(result.gestures) > 0:
            gesture = result.gestures[0][0].category_name

            # print(gesture, current_gesture, time.perf_counter() - gesture_time)
            if not dog_controller.action:
                if gesture != current_gesture:
                    gesture_time = time.perf_counter()
                    current_gesture = gesture

                if time.perf_counter() - gesture_time > required_time:
                    gesture_time = time.perf_counter()
                    if gesture == "Closed_Fist":
                        print("PRAY ACTIVATED")
                        dog_controller.pray()
                    elif gesture == "Pointing_Up":
                        print("DANCE ACTIVATED")
                        dog_controller.dance1()
            else:
                idle = False
                last_detection = time.perf_counter()
                gesture_time = time.perf_counter()

        # для создания image нужно раскоментировать 2 строки выше
        # cv.imshow("mp", image)
        cv.waitKey(1)
