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
from sh import tail


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

    def __init__(self, color, dim, length, width, area, hue, sat, val, center):
        self.color = color
        self.dim = dim
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
                + ' (' + str(round(self.dim[0], 1)) + '/' + str(round(self.dim[1], 1)) + ')'
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
        frame = self.reader.frame
        if frame is None:
            return [], None

        edges = cv2.Canny(frame, 100, 200)
        edges = cv2.dilate(edges, None, iterations=1)
        edges = cv2.erode(edges, None, iterations=1)
        ret, thresh = cv2.threshold(edges, 127, 255, cv2.THRESH_BINARY)
        mask, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
        for x in range(0, len(contours)):
            cv2.fillConvexPoly(mask, contours[x], 255)

        # run a second round on filled contours
        mask, contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)

        pieces = []
        for c in contours:
            p = self.__identify_piece(c, mask, frame)
            if p is not None:
                pieces.append(p)

        return pieces, frame

    @staticmethod
    def __identify_piece(contour, mask, frame):
        # center (x,y), (width, height), angle of rotation
        rect = cv2.minAreaRect(contour)
        center = rect[0]
        dim = rect[1]
        width = int(round(dim[0] / 9.5))
        length = int(round(dim[1] / 9.5))
        if width > length:
            width, length = length, width

        b = frame[:, :, 0]
        g = frame[:, :, 1]
        r = frame[:, :, 2]

        mask.fill(0)
        cv2.fillConvexPoly(mask, contour, 255)

        piece = None
        # calculate average colour of masked frame
        if np.sum(mask) != 0:
            avgB = np.average(b, axis=None, weights=mask)
            avgG = np.average(g, axis=None, weights=mask)
            avgR = np.average(r, axis=None, weights=mask)
            hsv = colorsys.rgb_to_hsv(avgR * 0.1 / 255, avgG * 0.1 / 255, avgB * 0.1 / 255)
            hue = round(hsv[0] * 360, 1)
            sat = round(hsv[1] * 100, 1)
            val = round(hsv[2] * 100, 1)
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
            piece = Piece(color, dim, length, width, area, hue, sat, val, center)

        return piece

    def read(self):
        try:
            return self.read_internal()
        except:
            return [], None

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
        self.freq = 800
        self.sleep = 1
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
        return int(round(time.time() + seconds))

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


class Listener:

    def __init__(self):
        pass

    def on_piece_detected(self, piece, image):
        pass


class Sorter:

    def __init__(self):
        self.lock = threading.Lock()
        self.running = True
        self.beltSpeed = 600
        self.beltLength = 400
        self.predicate = Sorter.__false
        self.listener = Listener()
        self.thread = None

    def set_predicate(self, predicate):
        self.predicate = predicate

    def set_listener(self, listener):
        self.listener = listener

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

                pieces, frame = cap.read()
                piece = None
                if len(pieces) > 0:
                    piece = pieces[0]

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
                            self.listener.on_piece_detected(piece, frame)
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
                            # s.non_match_in(t)

                time.sleep(.2)


class FilePredicate:

    def __init__(self, path):
        self.path = path
        self.colors = set()
        t = threading.Thread(None, self)
        t.setDaemon(True)
        t.start()

    def __call__(self, *args, **kwargs):
        try:
            self.__read()
        except Exception as e:
            print "Reading from command file failed: ", e

    def __read(self):
        for line in tail("-f", "-n", "0", self.path, _iter=True):
            print('command: ' + line)
            self.colors.clear()
            for c in line[len('lego-sorter:'):].strip().split():
                if c == Color.BLACK or c.startswith('schwarz'):
                    self.colors.add(Color.BLACK)
                elif c == Color.BLUE or c.startswith('blau'):
                    self.colors.add(Color.BLUE)
                elif c == Color.GRAY or c.startswith('grau'):
                    self.colors.add(Color.GRAY)
                elif c == Color.GREEN or c.startswith('gr'):
                    self.colors.add(Color.GREEN)
                elif c == Color.ORANGE or c.startswith('oran'):
                    self.colors.add(Color.ORANGE)
                elif c == Color.RED or c.startswith('rot'):
                    self.colors.add(Color.RED)
                elif c == Color.YELLOW or c.startswith('gelb'):
                    self.colors.add(Color.YELLOW)
            color_str = 'nothing'
            if len(self.colors) > 0:
                color_str = str(self.colors)
            print('Now sorting :' + color_str)

    def predicate(self, piece):
        return piece.color in self.colors
