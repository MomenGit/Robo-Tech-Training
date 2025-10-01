import numpy as np
import cv2 as cv


class DrawingCanvas:
    def __init__(self, width, height, bgr_color):
        self.width = width
        self.height = height
        self.bgr_color = bgr_color  # color in BGR format
        self.__init_canvas()

    def __init_canvas(self):
        self.__canvas = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        self.__canvas[:] = self.bgr_color
        return self.__canvas

    def set_canvas(self, canvas):
        self.__canvas = canvas

    def get_canvas(self):
        return self.__canvas

    def show_canvas(self):
        if self.__canvas is not None:
            cv.imshow("Drawing Canvas", self.__canvas)

    def decorate_canvas(self, text):
        font = cv.FONT_HERSHEY_SIMPLEX
        cv.putText(self.__canvas, text, (1, 1), font,
                   1, (255, 255, 255), 2, cv.LINE_AA)
