from template_finder import TemplateFinder
import cv2


class ImageProcessorTrack:
    def __init__(self):
        self.templ_finder = TemplateFinder()

        self.frame = None
        self.alliesTemplates = [None, None, None
        self.locAlliesTemplates = [[], [], []]

        self.enemiesTemplates = [None, None, None]
        self.locEnemiesTemplates = [[], [], []]

        self.ballTemplate = None
        self.locBallTemplate = []

        self.point_clicked = [None, None]

        self.processed_frame = None
        self.templatesExistence = False
    
    def process_frame(self, frame):
        self.frame = frame
        # TODO: Decide if update templates or not
        self.templ_finder.find_objects()

    def saveTemplates(self, CAMERA_INDEX):
        self.alliesTemplates[0] = cv2.imread("robot0.jpg")
        self.alliesTemplates[0] = cv2.imread("robot0.jpg")
        self.alliesTemplates[0] = cv2.imread("robot0.jpg")


    def get_mouse_clicks(self, event, x, y, flags, params):
        print("callback funcinou")
        if event == cv2.EVENT_LBUTTONDOWN:
            print(x, y)
            self.point_clicked = [x, y]

    def cropTemplate(self, template_type):
        pass

    def get_processed_frame(self):
        return self.processed_frame

    @property
    def thereAreTemplates(self):
        return self.templatesExistence