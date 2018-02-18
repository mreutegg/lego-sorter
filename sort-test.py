import lego
import time


def predicate(piece):
    return piece.color == lego.Color.RED

s = lego.Sorter()
try:
    s.set_predicate(predicate)
    s.start()
    # let it run for ten minutes
    time.sleep(600)
finally:
    s.stop()
