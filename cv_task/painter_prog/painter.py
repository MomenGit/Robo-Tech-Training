import math
import cv2 as cv
import numpy as np

from drawing_canvas import DrawingCanvas
from history_manager import HistoryManager


class Painter:
    """Class to handle different drawing modes on a canvas."""

    def __init__(self, drawing_canvas: DrawingCanvas, history_manager: HistoryManager):
        self.drawing = False  # true if mouse is pressed
        self.ix, self.iy = -1, -1

        self.drawing_canvas = drawing_canvas
        self.canvas = self.drawing_canvas.get_canvas()

        self.history_manager = history_manager
        self.history_manager.add_state(self.canvas)

        self.drawing_modes = {
            ord('c'): self.draw_circle,
            ord('r'): self.draw_rectangle,
            ord('p'): self.draw_polygon, }
        self.operations = {
            ord('e'): self.erase,
            ord('x'): self.crop,
            ord('a'): self.rotate,
            ord('z'): self.undo,
            ord('y'): self.redo,
        }

    def draw_rectangle(self, event, x, y, flags, param):
        """Draw rectangle on the canvas."""
        if event == cv.EVENT_LBUTTONDOWN:
            self.drawing = True
            self.ix, self.iy = x, y

        elif event == cv.EVENT_MOUSEMOVE:
            if self.drawing == True:
                cv.rectangle(self.canvas, (self.ix, self.iy),
                             (x, y), (0, 255, 0), -1)

        elif event == cv.EVENT_LBUTTONUP:
            self.drawing = False
            cv.rectangle(self.canvas, (self.ix, self.iy),
                         (x, y), (0, 255, 0), -1)
            self.history_manager.add_state(self.canvas)

    def draw_circle(self, event, x, y, flags, param):
        """Draw circle on the canvas."""
        radius = 5

        if event == cv.EVENT_LBUTTONDOWN:
            self.drawing = True
            self.ix, self.iy = x, y

        elif event == cv.EVENT_MOUSEMOVE:
            if self.drawing == True:
                dx = x - self.ix
                dy = y - self.iy
                radius = math.hypot(dx, dy)

        elif event == cv.EVENT_LBUTTONUP:
            self.drawing = False
            dx = x - self.ix
            dy = y - self.iy
            radius = int(math.hypot(dx, dy))
            cv.circle(self.canvas,
                      (self.ix, self.iy),
                      radius,
                      (0, 0, 255), -1)
            self.history_manager.add_state(self.canvas)

    def draw_polygon(self, event, x, y, flags, param):
        """Draw polygon on the canvas."""
        lines = 3
        if event == cv.EVENT_LBUTTONDOWN:
            self.drawing = True
            lines += 1
            cv.polylines(self.canvas, [np.array(
                [[self.ix, self.iy], [x, y]], np.int32)], False, (0, 255, 0), thickness=2)

        elif event == cv.EVENT_LBUTTONUP:
            self.drawing = False
            self.history_manager.add_state(self.canvas)

    def erase(self):
        pass

    def crop(self):
        pass

    def rotate(self):
        pass

    def undo(self):
        prev_canvas = self.history_manager.undo()
        if prev_canvas is not None:
            self.canvas = prev_canvas
            self.drawing_canvas.set_canvas(self.canvas)
            self.drawing_canvas.show_canvas()
            cv.waitKey(1)

    def redo(self):
        next_canvas = self.history_manager.redo()
        if next_canvas is not None:
            self.canvas = next_canvas
            self.drawing_canvas.set_canvas(self.canvas)
            self.drawing_canvas.show_canvas()
            cv.waitKey(1)
