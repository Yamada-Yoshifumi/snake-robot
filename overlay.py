import sys
import cv2
import numpy as np


def main():
    img_path = sys.argv[1]
    img = cv2.imread(img_path)

    w = 120
    h = 80
    mid_w = int(np.floor(w / 2))
    mid_h = int(np.floor(h / 2))

    candidate_points = []
    # Straight forward
    candidate_points.append(np.array([mid_h, mid_w]))
    # Left turn: spine_offset=-0.3
    candidate_points.append(np.array([mid_h, 7]))  # Manually calculated
    # Right turn: spine_offset=0.3
    candidate_points.append(np.array([mid_h, w-7-1]))  # To account for array index

    colours = []
    for i in range(3):
        if i == int(sys.argv[2]):
            colours.append((0,255,0))
        else:
            colours.append((0,0,255))

    for i in range(3):
        cv2.line(img,(mid_w, h-1),(candidate_points[i][1],candidate_points[i][0]),colours[i],2)  # Notice OpenCV Point revert col and row!

    # cv2.imshow("Image", img)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    output_path = img_path.replace("mask", "selection")
    cv2.imwrite(output_path, img)

    return

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print("Usage: python3 <img_path> <num_of_selected_trajectory>")
        exit(0)
    main()