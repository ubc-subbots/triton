import cv2 as cv
import numpy as np

DEBUG = True


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
  


src = cv.imread("./shapes.png")

segment = util_segment(src, 0)

morph = morphological(segment, (5,5), (5,5))

contours, hierarchy = cv.findContours(morph, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

print(f'Number of contours: {len(contours)}')

src_cnt = src.copy()
cv.drawContours(src_cnt, contours, -1, (0,255,0), 3)
cv.imshow("contours", src_cnt)
cv.waitKey(0)

i = 0
for cnt in contours:
    blank = np.zeros(src.shape, dtype="uint8")

    # Contour approximation
    approxC = cv.approxPolyDP(cnt, 3, True)
    print(f'Number of vertices: {len(approxC)}')

    # Area
    area = cv.contourArea(cnt)
    print(f'Area: {area}')

    # Perimeter
    perimeter = cv.arcLength(cnt, True)
    print(f'Perimeter: {perimeter}')

    # Bounding rect
    rect = cv.minAreaRect(cnt)
    box = cv.boxPoints(rect)
    box = np.int0(box)

    # Centroid
    moment = cv.moments(cnt)
    # calculate x,y coordinate of center
    cX = int(moment["m10"] / moment["m00"])
    cY = int(moment["m01"] / moment["m00"])

    # Circularity
    # (4*Area*pi)/(Perimeter^2)
    circularity = 4 * area * 3.14159 / (perimeter ** 2)
    print(f'Circularity: {circularity}')

    # Convex hull
    hull = cv.convexHull(cnt)


    # Ellipse
    ellipse = cv.fitEllipse(cnt)

    # center, axis_length and orientation of ellipse
    (center,axes,orientation) = ellipse

    # length of MAJOR and minor axis
    majoraxis_length = max(axes)
    minoraxis_length = min(axes)

    # eccentricity = sqrt( 1 - (ma/MA)^2)
    eccentricity = np.sqrt(1-(minoraxis_length/majoraxis_length)**2)
    print(f'Eccentricity: {eccentricity}')


    cv.drawContours(blank,[box],0,(255,255,0),1)
    cv.drawContours(blank, hull, -1, (0,0,255), 3)
    cv.drawContours(blank, approxC, -1, (0,255,0), 3)
    cv.circle(blank, (cX, cY), 3, (255, 0, 0), -1)

    cv.imshow(f'contour {i}', blank)
    cv.waitKey(0)
    i += 1

#closing all open windows 
cv.destroyAllWindows() 
