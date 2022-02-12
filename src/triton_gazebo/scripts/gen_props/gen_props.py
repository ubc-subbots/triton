import cv2 as cv
from cv2 import imread
import numpy as np
import random
import time

DEBUG = False

# Simple model testing using generated data: 
# https://colab.research.google.com/drive/1ioHqfHTLN5b0vslCkrpzNH2fUfuUStno?usp=sharing


def util_segment(src, hue):

    # Boundaries
    low_H = hue - 10
    low_S = 80
    low_V = 80
    high_H = hue + 10
    high_S = 255
    high_V = 255

    # Since the range of hue in OpenCV is 0 to 180
    if (low_H < 0):
        low_H += 180
    if (high_H > 180):
        high_H -= 180

    # Convert to HSV color space
    segment = cv.cvtColor(src, cv.COLOR_BGR2HSV)

    if (low_H > high_H):
        segment1 = cv.inRange(segment, (low_H, low_S, low_V), (180, high_S, high_V))
        segment2 = cv.inRange(segment, (0, low_S, low_V), (high_H, high_S, high_V))
        segment = cv.bitwise_or(segment1, segment2)
    else:
        segment = cv.inRange(segment, (low_H, low_S, low_V), (high_H, high_S, high_V))

    if (DEBUG):
        segmented_src = cv.bitwise_and(src, src, mask=segment)
        cv.imshow("window2", segment)
        cv.imshow("window1", segmented_src)

        cv.waitKey(0) 

    return segment

def morphological(src, open_kernel, close_kernel):
    open_k = cv.getStructuringElement(cv.MORPH_RECT, open_kernel)
    close_k = cv.getStructuringElement(cv.MORPH_RECT, close_kernel)
    opening = cv.morphologyEx(src, cv.MORPH_OPEN, open_k)
    closing = cv.morphologyEx(opening, cv.MORPH_CLOSE, close_k)
    if (DEBUG):
        cv.imshow("morph", closing)

        cv.waitKey(0) 
    return closing
  

# type: 0 = circle, 1 = star, 2 = glass
#
def create_image(type):
    if (type == 0):
        if (random.random() > 0.5):
            shape_img = cv.imread("./shapes/circle_1.png")
        else:
            shape_img = cv.imread("./shapes/circle_2.png")
    elif (type == 1):
        shape_img = cv.imread("./shapes/star.png")
    else: #elif (type == 2):
        shape_img = cv.imread("./shapes/glass.png")


    # Affine transformation
    # https://docs.opencv.org/3.4/d4/d61/tutorial_warp_affine.html
    r1 = random.uniform(0.0, 0.3)
    r2 = random.uniform(0.7, 1.0)
    r3 = random.uniform(0.0, 0.3)
    r4 = random.uniform(0.0, 0.3)
    r5 = random.uniform(0.7, 1.0)
    srcTri = np.array( [[0, 0], [shape_img.shape[1] - 1, 0], [0, shape_img.shape[0] - 1]] ).astype(np.float32)
    dstTri = np.array( [[0, shape_img.shape[1]*r1], [shape_img.shape[1]*r2, shape_img.shape[0]*r3], [shape_img.shape[1]*r4, shape_img.shape[0]*r5]] ).astype(np.float32)
    warp_mat = cv.getAffineTransform(srcTri, dstTri)
    shape_img = cv.warpAffine(shape_img, warp_mat, (shape_img.shape[1], shape_img.shape[0]), borderValue=(255,255,255))

    # Rotate shape
    height, width = shape_img.shape[:2]
    center = (width/2, height/2)

    rangle = random.randrange(-179, 180)
    rscale = random.uniform(0.8,1.5)
    rotate_matrix = cv.getRotationMatrix2D(center=center, angle=rangle, scale=1)

    shape_img = cv.warpAffine(src=shape_img, M=rotate_matrix, dsize=(width, height), borderValue=(255,255,255))

    # Create blank image
    canvas_img = np.zeros([512,512,3],dtype=np.uint8)
    canvas_img.fill(255)

    # Paste shape onto image
    canvas_img[128:128+shape_img.shape[1], 128:128+shape_img.shape[0], :] = shape_img

    # Create artificial blue tint
    rblueness = random.uniform(0.3,0.7)
    rbrightness = random.uniform(0.5,0.9)
    canvas_img[:,:, 1:] = canvas_img[:,:, 1:] * rblueness 
    canvas_img[:,:, 0] = canvas_img[:,:, 0] * rbrightness

    if (DEBUG):
        cv.imshow("Final image", canvas_img)
        cv.waitKey(0)

    return canvas_img

start_time = time.time()

# Get the target shapes we want
target_cnt = []
target_fn = ["./shapes/circle_pure.png", "./shapes/star_pure.png", "./shapes/glass_pure.png"]
for i in range(3):
    src = cv.imread(target_fn[i])
    segment = util_segment(src, 0)
    morph = morphological(segment, (2,2), (5,5))
    contours, hierarchy = cv.findContours(morph, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    if (DEBUG):
        ma = cv.matchShapes(contours[0], contours[1],1,0)
        print(ma)
    target_cnt.append(contours[0])

cv.waitKey(0)

# shape, number of vertices, circularity, contour size, eccentricity, extent, solidity, convexity, matchshape circle, matchshape star, matchshape glass
data = []

target_sample_size = 10000
sample_size = int(target_sample_size / 1.25)

print(f'Generating ~{target_sample_size} samples...')

shape = 0

shape_dist = [0,0,0,0]

not_a_shape_from_not_a_glass = 0

for iter in range(sample_size):
    print(f'Progress: {iter/sample_size * 100:.2f}%    Shape distribution: {shape_dist}', end="\r")
    if (iter > 0 and iter % int(sample_size / 3) == 0):
        shape += 1

    src = create_image(shape)

    segment = util_segment(src, 0)

    morph = morphological(segment, (2,2), (5,5))

    contours, hierarchy = cv.findContours(morph, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)


    src_cnt = src.copy()
    if (DEBUG):
        print(f'Number of contours: {len(contours)}')
        cv.drawContours(src_cnt, contours, -1, (0,255,0), 3)
        cv.imshow("contours", src_cnt)
        cv.waitKey(0)

    i = 0
    for cnt in contours:

        # Contour approximation
        approxC = cv.approxPolyDP(cnt, 3, True)

        # Area
        area = cv.contourArea(cnt)

        # Perimeter
        perimeter = cv.arcLength(cnt, True)

        # Bounding rect
        rect = cv.minAreaRect(cnt)
        box = cv.boxPoints(rect)
        box = np.int0(box)
        x,y,w,h = cv.boundingRect(cnt)

        # Extent 
        extent = float(area)/(w*h)

        # Centroid
        moment = cv.moments(cnt)
        # calculate x,y coordinate of center
        cX = int(moment["m10"] / moment["m00"])
        cY = int(moment["m01"] / moment["m00"])

        # Circularity
        # (4*Area*pi)/(Perimeter^2)
        circularity = 4 * area * 3.14159 / (perimeter ** 2)

        # Convex hull
        hull = cv.convexHull(cnt)
        hullarea = cv.contourArea(hull)

        # Solidity
        solidity = float(area)/hullarea

        # Convexity
        convexity = cv.isContourConvex(cnt)

        # Match shapes
        ma_c = cv.matchShapes(cnt, target_cnt[0],1,0)
        ma_s = cv.matchShapes(cnt, target_cnt[1],1,0)
        ma_g = cv.matchShapes(cnt, target_cnt[2],1,0)
        if (ma_c > 5):
            ma_c = 5
        if (ma_s > 5):
            ma_s = 5
        if (ma_g > 5):
            ma_g = 5

        # Convexity defects
        chull = cv.convexHull(cnt, returnPoints=False)
        defects = cv.convexityDefects(cnt, chull)
        if (defects is not None):
            num_defects = defects.shape[0]
        else:
            num_defects = 0


        # Ellipse
        # We filter by area because we know how big the shapes should be in our generated data
        if (cnt.size >= 10 and area > 1000):
            ellipse = cv.fitEllipse(cnt)

            # center, axis_length and orientation of ellipse
            (center,axes,orientation) = ellipse

            # length of MAJOR and minor axis
            majoraxis_length = max(axes)
            minoraxis_length = min(axes)

            # eccentricity = sqrt( 1 - (ma/MA)^2)
            eccentricity = np.sqrt(1-(minoraxis_length/majoraxis_length)**2)

            arr = [shape, len(approxC), circularity, cnt.size, eccentricity, extent, solidity, convexity, ma_c, ma_s, ma_g, num_defects]
            data.append(arr)

            shape_dist[shape] += 1

        else: 
            # Sometimes there are a lot of contours from the shot glass image
            if (shape_dist[3] < sample_size / 4 or shape_dist[3] < shape_dist[0] * 1.05):
                if (shape < 2):
                    #print(f"\n==========\nTHIS ONE {shape}\n========\n")
                    not_a_shape_from_not_a_glass += 1
                shape_dist[3] += 1
                eccentricity = 0
                arr = [3, len(approxC), circularity, cnt.size, eccentricity, extent, solidity, convexity, ma_c, ma_s, ma_g, num_defects]
                data.append(arr)


        if (DEBUG):
            print(f'Number of vertices: {len(approxC)}')
            print(f'Area: {area}')
            print(f'Perimeter: {perimeter}')
            print(f'Circularity: {circularity}')
            print(f'Contour size: {cnt.size}')
            print(f'Eccentricity: {eccentricity}')
            print(f'Extent: {extent}')
            print(f'Solidity: {solidity}')
            print(f'Convexity: {convexity}')
            print(f'Match shape: circle: {ma_c}')
            print(f'Match shape: star: {ma_s}')
            print(f'Match shape: glass: {ma_g}')
            print(f'Convexity defects: {num_defects}')

            blank = np.zeros(src.shape, dtype="uint8")
            cv.drawContours(blank,[box],0,(255,255,0),1)
            cv.drawContours(blank, hull, -1, (0,0,255), 3)
            cv.drawContours(blank, approxC, -1, (0,255,0), 3)
            cv.circle(blank, (cX, cY), 3, (255, 0, 0), -1)

            cv.imshow(f'contour {i}', blank)
            cv.waitKey(0)
            i += 1

    #closing all open windows 
    if (DEBUG):
        cv.destroyAllWindows() 

print(f'Progress: 100%    Shape distribution: {shape_dist}')

print(len(data))
datanp = np.array(data)
np.save("./data_11.npy", datanp)

print(f"Script took: {time.time() - start_time:.3f}s")
