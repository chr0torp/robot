import math
import numpy as np
import cv2
from sklearn.cluster import KMeans
from sklearn.cluster import DBSCAN


def angle_between_lines(arr: np.ndarray):
    """
    """
    final = []

    for i in arr:
        x1, y1, x2, y2 = i[0]
        x0 = x1
        y0 = y1 + 10

        point0 = np.array([x0, y0])
        point1 = np.array([x1, y1])
        point2 = np.array([x2, y2])

        a = point0 - point1
        b = point2 - point1

        ab = np.dot(a, b)
        abs_ab = np.linalg.norm(a) * np.linalg.norm(b)


        deg = math.degrees(math.acos(ab/abs_ab))

        if abs(deg-180) < deg:
            deg = abs(deg - 180)

        if deg < 15:
            final.append(i)
    
    return np.array(final)


def filter_canny_edges(lower: int, upper: int, img: np.ndarray):
    kernel = np.ones((7,7),np.float32)/20
    dst = cv2.filter2D(img,-1,kernel)
    gray = cv2.cvtColor(dst, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, lower, upper)
    return edges



def hh_lines(edges):
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 68, minLineLength=15, maxLineGap=250)
    return lines


def draw_lines(img, lines):
    """
    """
    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(img, (x1, y1), (x2, y2), (255, 0, 0), 3) 
        # print(f"Line drawn from ({x1}, {y1}) to ({x2}, {y2})")  


def K_means_clustering(arr: np.ndarray, n: int):
    """
    Perform K-means clustering on the input array.
    
    """

    arr = arr.reshape(-1, 4)[:, 0].reshape(-1, 1)
    kmeans = KMeans(n_clusters=n, random_state=0, n_init='auto')
    kmeans.fit(arr)

    original_labels = kmeans.labels_
    centroids = kmeans.cluster_centers_
    
    if n == 1:
        return original_labels

    centroid_values = centroids.flatten()
    sorted_centroid_indices = np.argsort(centroid_values)

    label_mapping = np.zeros(n, dtype=int)
    for new_label, original_cluster_index in enumerate(sorted_centroid_indices):
        label_mapping[original_cluster_index] = new_label

    new_labels = label_mapping[original_labels]

    return new_labels


def dbscan(arr: np.ndarray):
    """
    """
    
    arr = arr.reshape(-1, 4)[:, 0].reshape(-1, 1)

    db = DBSCAN(eps=80 , min_samples=3).fit(arr)
    db.fit(arr)
    original_labels = db.labels_


    uniq = set(original_labels)
    lowest = min(uniq)
    
    return list(original_labels), len(uniq)
    
    


def index(lst: list, value: int):
    """
    
    """

    list = [[] for i in range(value)]

    for i, v in enumerate(lst):
        list[v].append(i)
    return list

def points(arr: np.ndarray):
    """
    Extract points from the array.
    """
    arr = arr.reshape(-1, 4)[:, 0].reshape(-1, 1)
    point = arr.tolist()
    return point

# def sort_index(lst: list, points: list):
def sort_index(lst: list, arr: np.ndarray):
    # arr: np.ndarray
    """
    """

    point = points(arr)
    # print(f"points: {point}")

    n = len(lst)

    for i in range(n-1):

        for j in range(i, n):
            val1 = point[lst[i][0]][0]

            val2 = point[lst[j][0]][0]

            if val1 > val2:
                wait = lst[i]
                lst[i] = lst[j]
                lst[j] = wait
                print(f"Sorted lst: {lst}")

    return lst

def avg_hight(arr: np.ndarray):
    """
    """
    heights = []
    for line in arr:
        y1 = line[0][1]
        y2 = line[0][3]
        heights.append(max(y1, y2))

    if sum(heights) == 0 or len(heights) == 0:
        print("No heights detected, returning 0.")
        return -1
    
    avg = sum(heights) / len(heights)
    return avg

def run_height(img):

    lower = 50
    upper = lower * 2

    edges = filter_canny_edges(lower, upper, img)


    lines = hh_lines(edges)
    lines = angle_between_lines(lines)

    avg = avg_hight(lines)

    if lines is None or len(lines) == 0:
        print("No lines detected after angle filtering. Skipping clustering.")
        return -1
    
    return avg

def avg_center(arr: np.ndarray):
    """
    """
    centers = []
    for line in arr:
        x = line[0][0]
        centers.append(x)

    if sum(centers) == 0 or len(centers) == 0:
        print("No centers detected, returning -1.")
        return -1
    
    avg = sum(centers) / len(centers)
    return avg

def run_center(img):

    lower = 50
    upper = lower * 2

    edges = filter_canny_edges(lower, upper, img)


    lines = hh_lines(edges)
    lines = angle_between_lines(lines)

    avg = avg_center(lines)

    if lines is None or len(lines) == 0:
        print("No lines detected after angle filtering. Skipping clustering.")
        return -1
    
    return avg


def run(img):
    """
    """
    lower = 50
    upper = lower * 2

    edges = filter_canny_edges(lower, upper, img)


    lines = hh_lines(edges)
    lines = angle_between_lines(lines)

    if lines is None or len(lines) == 0:
        print("No lines detected after angle filtering. Skipping clustering.")
        return -1, -1, lines

    db, clustering = dbscan(lines)

    index_list = index(db, clustering)

    sorted_index = sort_index(index_list, lines)

    return clustering, sorted_index, lines




if __name__ == "__main__":
    # arr = np.array([[[2, 2, 2, 3]], [[2, 2, 3, 20]], [[2, 2, 3, 2]]])
    lst = [[0, 3, 4, 10], [1, 2, 6, 8, 9, 12, 13, 17], [7, 16], [5, 11, 14, 15]]
    points = [[474], [997], [1023], [482], [483], [1611], [999], [7], [1007], [996], [473], [1614], [998], [1000], [1652], [1652], [2], [1006]]

    lst2 = [[2, 3, 7, 10], [0, 1, 8, 13, 14, 15], [5, 6], [4, 9, 11, 12, 16]]
    points2 = [[999], [1007], [474], [483], [1608], [6], [4], [473], [998], [1649], [532], [1608], [1609], [995], [996], [997], [1650]]
    
    lst3 = [[0, 1, 11, 12, 13], [3, 4, 6, 9, 14], [5, 10], [2, 7, 8, 15]]
    points3 = [[997], [1006], [1608], [481], [482], [4], [473], [1634], [1633], [516], [1], [994], [997], [998], [529], [1627]]

    sorted_list = sort_index(lst, points)
    print("Sorted list:", sorted_list)
    print("lst", lst)
    print(f"points: {points}")

    sorted_list2 = sort_index(lst2, points2)
    print("Sorted list2:", sorted_list2)
    print("lst2", lst2)
    print(f"points2: {points2}")

    sorted_list3 = sort_index(lst3, points3)
    print("Sorted list3:", sorted_list3)
    print("lst3", lst3)
    print(f"points3: {points3}")

    