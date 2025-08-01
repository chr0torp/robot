import time
import math
import cv2

from detect_utils import *

def show_off_dbscan(img):
    """
    Show the results of DBSCAN clustering on the image.
    """
    lower = 50
    upper = lower * 2

    edges = filter_canny_edges(lower, upper, img)

    resized_edges = cv2.resize(edges, (0, 0), fx=0.5, fy=0.5)
    cv2.imshow("edges", resized_edges)
    cv2.imwrite("edges.png", resized_edges)

    img_with_hough_lines = img.copy()
    lines = hh_lines(edges)
    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(img_with_hough_lines, (x1, y1), (x2, y2), (0, 255, 0), 1)
    resized_lines = cv2.resize(img_with_hough_lines, (0, 0), fx=0.5, fy=0.5)
    cv2.imshow("lines", resized_lines)
    cv2.imwrite("lines.png", resized_lines)

    img_with_angle_lines = img.copy()
    lines = angle_between_lines(lines)
    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(img_with_angle_lines, (x1, y1), (x2, y2), (255, 0, 0), 1)
    resized_angle_lines = cv2.resize(img_with_angle_lines, (0, 0), fx=0.5, fy=0.5)
    cv2.imshow("angle lines", resized_angle_lines)
    cv2.imwrite("angle_lines.png", resized_angle_lines)


    db, clustering = dbscan(lines)
    show_lines = lines.reshape(-1, 4)[:, 0].reshape(-1, 1)
    print(f"before DBSCAN points: {show_lines.flatten().tolist()}")
    print(f"DBSCAN clustering result: {db}")
    print(f"Clustering labels: {clustering}")
    
    img_with_dbscan_results = img.copy()
    for i, line in enumerate(lines):
        x1, y1, x2, y2 = line[0]
        label = db[i]
        color = (0, 0, 255) if label == -1 else (255, 0, 0)
        cv2.line(img_with_dbscan_results, (x1, y1), (x2, y2), color, 1)
    resized_final_db = cv2.resize(img_with_dbscan_results, (0, 0), fx=0.5, fy=0.5)
    cv2.imshow("DBSCAN Result", resized_final_db)
    cv2.imwrite("dbscan_result.png", resized_final_db)

    index_list = index(db, clustering)
    print(f"Index list: {index_list}")

    sorted_index = sort_index(index_list, lines)
    print(f"Sorted index: {sorted_index}")

    cv2.waitKey(0)
    cv2.destroyAllWindows()



path_image = r"C:\Users\chris\Desktop\ba\pictures1\show.png"
img_data = cv2.imread(path_image)

show_off_dbscan(img_data)
