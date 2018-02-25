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
import lego
import time
import cv2
import os


class ImageWriter(lego.Listener):

    def on_piece_detected(self, piece, image):
        path = 'images/' + piece.color + '/' + str(piece.length) + 'x' + str(piece.width)
        if not os.access(path, os.F_OK):
            os.makedirs(path)
        name = str(int(round(time.time() * 1000))) + '.png'
        cv2.imwrite(path + '/' + name, image)


s = lego.Sorter()
try:
    p = lego.FilePredicate('/home/pi/legogram/command.log')
    s.set_predicate(p.predicate)
    s.set_listener(ImageWriter())
    s.start()
    # let it run for ten minutes
    time.sleep(600)
except KeyboardInterrupt:
    pass
finally:
    s.stop()
