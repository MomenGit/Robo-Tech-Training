import cv2 as cv
import numpy as np
from drawing_canvas import DrawingCanvas
from painter import Painter
from history_manager import HistoryManager


def main():
    # Step 1: Get dimensions from user
    width = int(input("Enter image width: "))
    height = int(input("Enter image height: "))

    # Step 2: Get color in BGR format
    print("Enter color in BGR format (0-255 each).")
    b = int(input("Enter Blue value: "))
    g = int(input("Enter Green value: "))
    r = int(input("Enter Red value: "))

    drawingCanvas = DrawingCanvas(width, height, (b, g, r))
    drawingCanvas.show_canvas()
    history_manager = HistoryManager()
    painter = Painter(drawingCanvas, history_manager)

    # Set initial mouse callback
    # Set default mode
    cv.setMouseCallback('Drawing Canvas', painter.draw_circle)

    while True:
        # Continuously update the window
        cv.imshow('Drawing Canvas', drawingCanvas.get_canvas())
        k = cv.waitKey(1) & 0xFF
        if k in painter.drawing_modes.keys():
            cv.setMouseCallback('Drawing Canvas', painter.drawing_modes[k])
        elif k in painter.operations.keys():
            painter.operations[k]()
        elif k == 27:  # ESC key to exit
            break

    cv.destroyAllWindows()


if __name__ == "__main__":
    main()
