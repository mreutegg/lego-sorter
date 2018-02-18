# Copyright 2018 Marcel Reutegger
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import threading
import colorsys
import cv2
import numpy as np
import pigpio
import time
import heapq
import Queue


class Reader:

    def __init__(self, cap):
        self.cap = cap
        self.frame = None
        self.stop = False

    def __call__(self):
        while not self.stop:
            try:
                ret, self.frame = self.cap.read()
            except Exception as e:
                print "Reading frame failed: ", e


class Piece:

    def __init__(self, color, length, width, area, hue, sat, val, center):
        self.color = color
        self.length = length
        self.width = width
        self.area = area
        self.hue = hue
        self.sat = sat
        self.val = val
        self.center = center
        self.timestamp = time.time()

    def is_similar_to(self, other):
        if not isinstance(other, Piece):
            return False
        return self.color == other.color and self.__similar(self.length, other.length) and self.__similar(self.width, other.width)

    def velocity(self, other):
        if not isinstance(other, Piece):
            return None
        dist = other.center[1] - self.center[1]
        t = other.timestamp - self.timestamp
        return dist / t

    @staticmethod
    def __similar(v1, v2):
        return abs(v1 - v2) <= 1

    def __str__(self):
        return (self.color
                + ', ' + str(self.length) + 'x' + str(self.width)
                + ', p=' + str(self.center[1])
                + ', h=' + str(self.hue)
                + ', s=' + str(self.sat)
                + ', v=' + str(self.val))

class Color:

    def __init__(self):
        pass

    BLACK = 'black'
    GRAY = 'gray'
    RED = 'red'
    ORANGE = 'orange'
    YELLOW = 'yellow'
    GREEN = 'green'
    BLUE = 'blue'


class Capture:

    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        self.rows = 240
        self.cols = 320
        self.shape = (self.rows, self.cols, 3)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.shape[0])
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.shape[1])
        self.img = np.full(tuple([self.shape[0] * 2, self.shape[1] * 2, self.shape[2]]), 0, np.uint8)
        self.reader = Reader(self.cap)
        self.readerThread = threading.Thread(None, self.reader)
        self.readerThread.start()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def read_internal(self):
        piece = None

        frame = self.reader.frame
        if frame is None:
            return piece

        edges = cv2.Canny(frame, 100, 200)
        edges = cv2.dilate(edges, None, iterations=1)
        edges = cv2.erode(edges, None, iterations=1)
        ret, thresh = cv2.threshold(edges, 127, 255, cv2.THRESH_BINARY)
        mask, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        angle = 0
        length = 0
        width = 0
        center = None
        for x in range(0, len(contours)):
            cv2.fillConvexPoly(mask, contours[x], 255)

        if len(contours) > 0:
            contours = np.concatenate(contours)
        if len(contours) > 5:
            # TODO: replace with cv2.minAreaRect()?
            ellipse = cv2.fitEllipse(contours)
            center = ellipse[0]
            rect = ellipse[1]
            angle = ellipse[2]
            width = int(round(rect[0] / 11.0))
            length = int(round(rect[1] / 11.0))

        dst = mask
        if angle != 0:
            M = cv2.getRotationMatrix2D((self.cols / 2, self.rows / 2), angle + 90, 1)
            dst = cv2.warpAffine(mask, M, (self.cols, self.rows))

        self.img[:self.shape[0], :self.shape[1]] = frame
        b = frame[:, :, 0]
        g = frame[:, :, 1]
        r = frame[:, :, 2]
        self.img[:self.shape[0], -self.shape[1]:, 0] = mask
        self.img[-self.shape[0]:, -self.shape[1]:, 1] = edges
        self.img[-self.shape[0]:, :self.shape[1], 2] = dst

        # calculate average colour of masked frame
        if np.sum(mask) != 0:
            avgB = np.average(b, axis=None, weights=mask)
            avgG = np.average(g, axis=None, weights=mask)
            avgR = np.average(r, axis=None, weights=mask)
            hsv = colorsys.rgb_to_hsv(avgR * 0.1 / 255, avgG * 0.1 / 255, avgB * 0.1 / 255)
            hue = round(hsv[0] * 360, 1)
            sat = round(hsv[1] * 100, 1)
            val = round(hsv[2] * 100, 1)
            hsvStr = ', '.join(map(str, [hue, sat, val]))
            # print([hue, sat, val])
            area = np.count_nonzero(mask)
            color = ''
            if sat < 30:
                if val < 5:
                    color = Color.BLACK
                else:
                    color = Color.GRAY
            elif hue >= 300 or hue < 10:
                color = Color.RED
            elif 10. <= hue < 40:
                color = Color.ORANGE
            elif 40. <= hue < 90:
                color = Color.YELLOW
            elif 90. <= hue < 180:
                color = Color.GREEN
            elif 180. <= hue < 300:
                color = Color.BLUE
            piece = Piece(color, length, width, area, hue, sat, val, center)

        return piece

    def read(self):
        try:
            return self.read_internal()
        except:
            return None

    def close(self):
        self.reader.stop = True
        self.readerThread.join()
        self.cap.release()


class ConveyorBelt:

    def __init__(self, gpio=18):
        self.pwm_duty = 500000
        self.gpio = gpio
        self.pi = pigpio.pi()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def run_at(self, velocity):
        self.pi.hardware_PWM(self.gpio, velocity, self.pwm_duty)

    def close(self):
        if self.pi is not None:
            self.run_at(0)
            self.pi.stop()


class SeeSaw:

    class CommandExecutor:

        def __init__(self, seesaw, commands, lock):
            self.seesaw = seesaw
            self.commands = commands
            self.lock = lock
            self.stop_queue = Queue.Queue()

        def __call__(self, *args, **kwargs):
            while self.stop_queue.empty():
                c = None
                with self.lock:
                    c = self.get_command()
                self.process_one(c)
                if c is None:
                    time.sleep(0.1)

        def get_command(self):
            if len(self.commands) != 0:
                c = self.commands[0]
                if c[0] < int(time.time()):
                    return heapq.heappop(self.commands)
            return None

        def process_one(self, command):
            if command is None:
                return
            print('processing command ' + str(command))
            if command[1] == SeeSaw.LEFT:
                self.seesaw.left()
            else:
                self.seesaw.right()

        def stop(self):
            self.stop_queue.put('stop')

    LEFT = 'left'
    RIGHT = 'right'

    def __init__(self, gpio=19):
        self.side = None
        self.pwm_duty = 500000
        self.freq = 400
        self.sleep = 2
        self.gpio_dir = 26
        self.gpio = gpio
        self.pi = pigpio.pi()
        self.pi.set_mode(self.gpio_dir, pigpio.OUTPUT)
        self.lock = threading.Lock()
        self.commands = []
        self.processor = SeeSaw.CommandExecutor(self, self.commands, self.lock)
        self.processorThread = threading.Thread(None, self.processor)
        self.processorThread.start()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def match_in(self, seconds):
        self.__command_in(seconds, SeeSaw.LEFT)

    def non_match_in(self, seconds):
        self.__command_in(seconds, SeeSaw.RIGHT)

    def __command_in(self, seconds, side):
        t = self.__command_stamp(seconds)
        with self.lock:
            found = None
            for i in range(0, len(self.commands)):
                c = self.commands[i]
                if c[0] == t:
                    found = c
                    # there's already a command with this timestamp
                    if c[1] != side:
                        # turn into non-match
                        self.commands[i] = (t, SeeSaw.RIGHT)
                    break
            if found is None:
                heapq.heappush(self.commands, (t, side))

    @staticmethod
    def __command_stamp(seconds):
        return int((time.time() + seconds) / 2) * 2

    def left(self):
        if self.side == SeeSaw.LEFT:
            return
        self.pi.write(self.gpio_dir, 0)
        self.__move()
        self.side = SeeSaw.LEFT

    def right(self):
        if self.side == SeeSaw.RIGHT:
            return
        self.pi.write(self.gpio_dir, 1)
        self.__move()
        self.side = SeeSaw.RIGHT

    def __move(self):
        self.pi.hardware_PWM(self.gpio, self.freq, self.pwm_duty)
        time.sleep(self.sleep)
        self.pi.hardware_PWM(self.gpio, 0, self.pwm_duty)

    def close(self):
        self.processor.stop()
        self.processorThread.join()
        if self.pi is not None:
            self.pi.stop()


class Sorter:

    def __init__(self):
        self.lock = threading.Lock()
        self.running = True
        self.beltSpeed = 600
        self.beltLength = 400
        self.predicate = Sorter.__false
        self.thread = None

    def set_predicate(self, predicate):
        self.predicate = predicate

    @staticmethod
    def __false(piece):
        return False

    def start(self):
        if self.thread is not None:
            raise Exception('already started')
        self.thread = threading.Thread(None, self)
        self.thread.start()

    def stop(self):
        if self.thread is None:
            return
        with self.lock:
            self.running = False
        self.thread.join()

    def __call__(self, *args, **kwargs):
        with Capture() as cap, ConveyorBelt() as belt, SeeSaw() as s:
            # start belt
            belt.run_at(self.beltSpeed)
            match = 0
            previous = None
            # run until stopped
            while True:
                with self.lock:
                    if not self.running:
                        break

                piece = cap.read()
                if piece is None:
                    match = 0
                    previous = None
                else:
                    v = piece.velocity(previous)
                    currentPos = int(piece.center[1])
                    if currentPos < 50 or currentPos > 190:
                        # ignore positions close to border
                        # piece may be truncated
                        pass
                    elif piece.is_similar_to(previous):
                        # piece is similar to previous
                        match += 1
                        if match == 2:
                            # similar piece seen three times
                            # calculate when it will drop off the belt
                            t = (self.beltLength - currentPos) / v
                            print(piece.color + ' ' + str(piece.length)
                                  + 'x' + str(piece.width)
                                  + ' @ v=' + str(int(v))
                                  + ' (' + str(currentPos) + '),'
                                  + ' dropOffIn=' + str(round(t, 1)))
                            if self.predicate(piece):
                                s.match_in(t)
                            else:
                                s.non_match_in(t)
                            # reset match count
                            match = 0
                            previous = piece
                    else:
                        # piece is not similar to previous
                        if previous is not None:
                            print('not similar: previous=' + str(previous) + ', piece=' + str(piece))
                        match = 0
                        previous = piece
                        if v is not None:
                            # calculate when it will drop off the belt
                            t = (self.beltLength - currentPos) / v
                            s.non_match_in(t)

                time.sleep(.2)
