import json
import cv2
import numpy as np
import sys

#TODO: bug found if not configure all colors

KERNEL_DIMENSION = 7
CAMERA_INDEX = 0


# -------------------- DECLARATION OF VARIABLES SECTION  --------------------
config_points = []
segm_limits = [[[[0, 0, 0], [0, 0, 0], [0, 0, 0]], [[0, 0, 0], [0, 0, 0], [0, 0, 0]]],
              [[[0, 0, 0], [0, 0, 0], [0, 0, 0]], [[0, 0, 0], [0, 0, 0], [0, 0, 0]]],
              [[[0, 0, 0], [0, 0, 0], [0, 0, 0]], [[0, 0, 0], [0, 0, 0], [0, 0, 0]]],
              [[[0, 0, 0], [0, 0, 0], [0, 0, 0]], [[0, 0, 0], [0, 0, 0], [0, 0, 0]]],
              [[[0, 0, 0], [0, 0, 0], [0, 0, 0]], [[0, 0, 0], [0, 0, 0], [0, 0, 0]]],
              [[[0, 0, 0], [0, 0, 0], [0, 0, 0]], [[0, 0, 0], [0, 0, 0], [0, 0, 0]]]]


with open("config.json") as f:
    config_file = json.load(f)

    # Get hsv seetings for image segmentation
    # > Bottom limits
    segm_limits[0][0][0] = config_file['hsv_bottom_limit']['ball']['hue']
    segm_limits[0][0][1] = config_file['hsv_bottom_limit']['ball']['saturation']
    segm_limits[0][0][2] = config_file['hsv_bottom_limit']['ball']['value']

    segm_limits[1][0][0] = config_file['hsv_bottom_limit']['allie_0']['hue']
    segm_limits[1][0][1] = config_file['hsv_bottom_limit']['allie_0']['saturation']
    segm_limits[1][0][2] = config_file['hsv_bottom_limit']['allie_0']['value']

    segm_limits[2][0][0] = config_file['hsv_bottom_limit']['allie_1']['hue']
    segm_limits[2][0][1] = config_file['hsv_bottom_limit']['allie_1']['saturation']
    segm_limits[2][0][2] = config_file['hsv_bottom_limit']['allie_1']['value']

    segm_limits[3][0][0] = config_file['hsv_bottom_limit']['allie_2']['hue']
    segm_limits[3][0][1] = config_file['hsv_bottom_limit']['allie_2']['saturation']
    segm_limits[3][0][2] = config_file['hsv_bottom_limit']['allie_2']['value']

    segm_limits[4][0][0] = config_file['hsv_bottom_limit']['allie_shirt']['hue']
    segm_limits[4][0][1] = config_file['hsv_bottom_limit']['allie_shirt']['saturation']
    segm_limits[4][0][2] = config_file['hsv_bottom_limit']['allie_shirt']['value']

    segm_limits[5][0][0] = config_file['hsv_bottom_limit']['enemy_shirt']['hue']
    segm_limits[5][0][1] = config_file['hsv_bottom_limit']['enemy_shirt']['saturation']
    segm_limits[5][0][2] = config_file['hsv_bottom_limit']['enemy_shirt']['value']

    # > Top limits
    segm_limits[0][1][0] = config_file['hsv_top_limit']['ball']['hue']
    segm_limits[0][1][1] = config_file['hsv_top_limit']['ball']['saturation']
    segm_limits[0][1][2] = config_file['hsv_top_limit']['ball']['value']

    segm_limits[1][1][0] = config_file['hsv_top_limit']['allie_0']['hue']
    segm_limits[1][1][1] = config_file['hsv_top_limit']['allie_0']['saturation']
    segm_limits[1][1][2] = config_file['hsv_top_limit']['allie_0']['value']

    segm_limits[2][1][0] = config_file['hsv_top_limit']['allie_1']['hue']
    segm_limits[2][1][1] = config_file['hsv_top_limit']['allie_1']['saturation']
    segm_limits[2][1][2] = config_file['hsv_top_limit']['allie_1']['value']

    segm_limits[3][1][0] = config_file['hsv_top_limit']['allie_2']['hue']
    segm_limits[3][1][1] = config_file['hsv_top_limit']['allie_2']['saturation']
    segm_limits[3][1][2] = config_file['hsv_top_limit']['allie_2']['value']

    segm_limits[4][1][0] = config_file['hsv_top_limit']['allie_shirt']['hue']
    segm_limits[4][1][1] = config_file['hsv_top_limit']['allie_shirt']['saturation']
    segm_limits[4][1][2] = config_file['hsv_top_limit']['allie_shirt']['value']

    segm_limits[5][1][0] = config_file['hsv_top_limit']['enemy_shirt']['hue']
    segm_limits[5][1][1] = config_file['hsv_top_limit']['enemy_shirt']['saturation']
    segm_limits[5][1][2] = config_file['hsv_top_limit']['enemy_shirt']['value']

b_limits = segm_limits[0] # List that stores the hsv limits for ball segmentation; 0 pos in segm_limits list
a0_limits = segm_limits[1] # List that stores the hsv limits for first allie segmentation; 1 pos in segm_limits list
a1_limits = segm_limits[2] # List that stores the hsv limits for second allie segmentation; 2 pos in segm_limits list
a2_limits = segm_limits[3] # List that stores the hsv limits for third allie segmentation; 3 pos in segm_limits list
aS_limits = segm_limits[4] # List that stores the hsv limits for allie shirt segmentation; 4 pos in segm_limits list
eS_limits = segm_limits[5] # List that stores the hsv limits for enemy shirt segmentation; 5 pos in segm_limits list

kernel = (KERNEL_DIMENSION, KERNEL_DIMENSION)



with open('config.json') as f:
	config_file = json.load(f)

# -------------------- FUNCTIONS SECTION  --------------------
def save_settings():
    global config_file

    # Saving points for image transformation; config_points[point][x or y]
    for point_index in xrange(4):
        config_file['image_transformation']['point'][point_index]['x'] = config_points[point_index][0]
        config_file['image_transformation']['point'][point_index]['y'] = config_points[point_index][1]

    # Saving hsv settings for image segmentation segm_limits[config_for_each_color][bottom or top limit][h or s or v]

    config_file['hsv_bottom_limit']['ball']['hue'] = segm_limits[0][0][0]
    config_file['hsv_bottom_limit']['ball']['saturation'] = segm_limits[0][0][1]
    config_file['hsv_bottom_limit']['ball']['value'] = segm_limits[0][0][2]

    config_file['hsv_bottom_limit']['allie_0']['hue'] = segm_limits[1][0][0]
    config_file['hsv_bottom_limit']['allie_0']['saturation'] = segm_limits[1][0][1]
    config_file['hsv_bottom_limit']['allie_0']['value'] = segm_limits[1][0][2]

    config_file['hsv_bottom_limit']['allie_1']['hue'] = segm_limits[2][0][0]
    config_file['hsv_bottom_limit']['allie_1']['saturation'] = segm_limits[2][0][1]
    config_file['hsv_bottom_limit']['allie_1']['value'] = segm_limits[2][0][2]

    config_file['hsv_bottom_limit']['allie_2']['hue'] = segm_limits[3][0][0]
    config_file['hsv_bottom_limit']['allie_2']['saturation'] = segm_limits[3][0][1]
    config_file['hsv_bottom_limit']['allie_2']['value'] = segm_limits[3][0][2]

    config_file['hsv_bottom_limit']['allie_shirt']['hue'] = segm_limits[4][0][0]
    config_file['hsv_bottom_limit']['allie_shirt']['saturation'] = segm_limits[4][0][1]
    config_file['hsv_bottom_limit']['allie_shirt']['value'] = segm_limits[4][0][2]

    config_file['hsv_bottom_limit']['enemy_shirt']['hue'] = segm_limits[5][0][0]
    config_file['hsv_bottom_limit']['enemy_shirt']['saturation'] = segm_limits[5][0][2]
    config_file['hsv_bottom_limit']['enemy_shirt']['value'] = segm_limits[5][0][2]


    config_file['hsv_top_limit']['ball']['hue'] = segm_limits[0][1][0]
    config_file['hsv_top_limit']['ball']['saturation'] = segm_limits[0][1][1]
    config_file['hsv_top_limit']['ball']['value'] = segm_limits[0][1][2]

    config_file['hsv_top_limit']['allie_0']['hue'] = segm_limits[1][1][0]
    config_file['hsv_top_limit']['allie_0']['saturation'] = segm_limits[1][1][1]
    config_file['hsv_top_limit']['allie_0']['value'] = segm_limits[1][1][2]

    config_file['hsv_top_limit']['allie_1']['hue'] = segm_limits[2][1][0]
    config_file['hsv_top_limit']['allie_1']['saturation'] = segm_limits[2][1][1]
    config_file['hsv_top_limit']['allie_1']['value'] = segm_limits[2][1][2]

    config_file['hsv_top_limit']['allie_2']['hue'] = segm_limits[3][1][0]
    config_file['hsv_top_limit']['allie_2']['saturation'] = segm_limits[3][1][1]
    config_file['hsv_top_limit']['allie_2']['value'] = segm_limits[3][1][2]

    config_file['hsv_top_limit']['allie_shirt']['hue'] = segm_limits[4][1][0]
    config_file['hsv_top_limit']['allie_shirt']['saturation'] = segm_limits[4][1][1]
    config_file['hsv_top_limit']['allie_shirt']['value'] = segm_limits[4][1][2]

    config_file['hsv_top_limit']['enemy_shirt']['hue'] = segm_limits[5][1][0]
    config_file['hsv_top_limit']['enemy_shirt']['saturation'] = segm_limits[5][1][2]
    config_file['hsv_top_limit']['enemy_shirt']['value'] = segm_limits[5][1][2]

    with open("config.json", 'w') as f:
        json.dump(config_file, f, indent=4)

def nothing(argument):
    pass

def get_mouse_clicks(event, x, y, flags, params):
    global config_points
    if event == cv2.EVENT_LBUTTONDOWN:
        print "Point clicked: ({},{})".format(x, y)
        config_points.append([x,y])

def configure_transformation():
    print "\n===== STARTING IMAGE TRANSFORMATION CONFIGURATION ====="

    global config_points

    cap = cv2.VideoCapture(CAMERA_INDEX)

    if cap.isOpened() == False:
        print "Error while trying to open video input. Check your webcam or file and try again."
        exit()

    cv2.namedWindow("frame")

    cv2.setMouseCallback("frame", get_mouse_clicks)

    while True:
        ret, frame = cap.read()

        cv2.imshow("frame", frame)

        key = cv2.waitKey(1)

        if key == ord('q') or len(config_points) >= 4:
            print "\n>> POINTS TO GEOMETRICAL TRANSFORMATION SET SUCCESFULLY!"
            print "Points: {}".format(config_points)
            cap.release()
            cv2.destroyAllWindows()
            break

def configure_segmentation():
    print "\n===== STARTING IMAGE SEGMENTATION CONFIGURATION ====="

    global segm_limits

    global b_limits
    global a0_limits
    global a1_limits
    global a2_limits
    global aS_limits
    global eS_limits


    cap = cv2.VideoCapture(CAMERA_INDEX)

    bottom_h = 0
    bottom_s = 0
    bottom_v = 0
    top_h = 0
    top_s = 0
    top_v = 0

    cv2.namedWindow("mask")

    #Definition of trackbars for configuration
    cv2.createTrackbar('Bottom_H', "mask", 0, 360, nothing)
    cv2.createTrackbar('Top_H', "mask", 0, 360, nothing)

    cv2.createTrackbar('Bottom_S', "mask", 0, 360, nothing)
    cv2.createTrackbar('Top_S', "mask", 0, 360, nothing)

    cv2.createTrackbar('Bottom_V', "mask", 0, 360, nothing)
    cv2.createTrackbar('Top_V', "mask", 0, 360, nothing)

    while True:
        ret, frame = cap.read()

        #Convert our frame from BGR to HSV
        #hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.GaussianBlur(frame, (5,5), 0)
        mask = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


        #Define our range of colors for segmentation
        bottom_limit = np.array([bottom_h, bottom_s, bottom_v])
        top_limit = np.array([top_h, top_s, top_v])

        #Define a mask image of colours inside the range set by bottom_limit and top_limit
        #mask = cv2.inRange(hsv, bottom_limit, top_limit)
        frame = cv2.GaussianBlur(frame, kernel, 0)
        mask = cv2.GaussianBlur(mask, kernel, 0)
        mask = cv2.inRange(frame, bottom_limit, top_limit)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        cv2.imshow("frame", frame)
        #cv2.imshow("hsv", hsv)
        #cv2.imshow("opening", opening)
        cv2.imshow("mask", mask)

        #Get current position of our trackbars
        bottom_h = cv2.getTrackbarPos('Bottom_H', "mask")
        bottom_s = cv2.getTrackbarPos('Bottom_S', "mask")
        bottom_v = cv2.getTrackbarPos('Bottom_V', "mask")

        top_h = cv2.getTrackbarPos('Top_H', "mask")
        top_s = cv2.getTrackbarPos('Top_S', "mask")
        top_v = cv2.getTrackbarPos('Top_V', "mask")

        key = cv2.waitKey(1)

        if key == ord('q'):
            segm_limits = [b_limits, a0_limits, a1_limits, a2_limits, aS_limits, eS_limits]
            print segm_limits
            cap.release()
            cv2.destroyAllWindows()
            break

        elif key == ord('b'):
            print "\nConfiguring segmentation...\n"
            print "Bottom limit: {}".format(bottom_limit)
            print "Top limit: {}".format(top_limit)

            b_limits = [bottom_limit, top_limit]

            print "\nBall config set succesfully!\n---------------"

        elif key == ord('0'):
            print "\nConfiguring segmentation...\n"
            print "Bottom limit: {}".format(bottom_limit)
            print "Top limit: {}".format(top_limit)

            a0_limits = [bottom_limit, top_limit]

            print "\nFirst allie config set succesfully!\n---------------"

        elif key == ord('1'):
            print "\nConfiguring segmentation...\n"
            print "Bottom limit: {}".format(bottom_limit)
            print "Top limit: {}".format(top_limit)

            a1_limits = [bottom_limit, top_limit]

            print "\nSecond allie config set succesfully!\n---------------"

        elif key == ord('2'):
            print "\nConfiguring segmentation...\n"
            print "Bottom limit: {}".format(bottom_limit)
            print "Top limit: {}".format(top_limit)

            a2_limits = [bottom_limit, top_limit]

            print "\nThird allie config set succesfully!\n---------------"

        elif key == ord('a'):
            print "\nConfiguring segmentation...\n"
            print "Bottom limit: {}".format(bottom_limit)
            print "Top limit: {}".format(top_limit)

            aS_limits = [bottom_limit, top_limit]

            print "\nAllie shirt config set succesfully!\n---------------"

        elif key == ord('e'):
            print "\nConfiguring segmentation...\n"
            print "Bottom limit: {}".format(bottom_limit)
            print "Top limit: {}".format(top_limit)

            eS_limits = [bottom_limit, top_limit]

            print "\nEnemy shirt config set succesfully!\n---------------"

# -------------------- MAIN SECTION  --------------------

if __name__ == '__main__':

    if len(sys.argv) > 1:
        if sys.argv[1] == 'segm':
            configure_segmentation()
        elif sys.argv[1] == 'crop':
            configure_transformation()
        else:
            print 'No argument valid! Try again with "segm" or "crop"'
            exit()
    else:
        configure_transformation()
        configure_segmentation()


    print "\nSaving all configs..."
    save_settings()
    print "All settings set in config.json!\n\nPress 'Q' to exit the program."

    cv2.namedWindow("exit")
    key = cv2.waitKey(0)

    if key == ord('q'):
        print "\nBye!"

    exit()
