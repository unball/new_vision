import json
from image_preparer import ImagePreparer

# return a messsage to be assembled and published
class ImageProcessor:
    def __init__(self):
        self.frame = None
        self.processed_frame = None
        self.config_points = [[0,0], [0,0], [0,0], [0,0]]
        self.segm_limits = [[[0,0,0],[0,0,0]], [[0,0,0],[0,0,0]], [[0,0,0],[0,0,0]], [[0,0,0],[0,0,0]], [[0,0,0],[0,0,0]], [[0,0,0],[0,0,0]] ]

        self.get_settings()

        self.preparer = ImagePreparer(self.config_points)
        #tracker = ColorTracker()

    def get_settings(self):
        with open("config.json") as f:
            config_file = json.load(f)

            # get points for geometrical transformation
            self.config_points[0][0] = config_file['image_transformation']['point'][0]['x']
            self.config_points[0][1] = config_file['image_transformation']['point'][0]['y']

            self.config_points[1][0] = config_file['image_transformation']['point'][1]['x']
            self.config_points[1][1] = config_file['image_transformation']['point'][1]['y']

            self.config_points[2][0] = config_file['image_transformation']['point'][2]['x']
            self.config_points[2][1] = config_file['image_transformation']['point'][2]['y']

            self.config_points[3][0] = config_file['image_transformation']['point'][3]['x']
            self.config_points[3][1] = config_file['image_transformation']['point'][3]['y']

            # get hsv seetings for image segmentation
            # > Bottom limits
            self.segm_limits[0][0][0] = config_file['hsv_bottom_limit']['ball']['hue']
            self.segm_limits[0][0][1] = config_file['hsv_bottom_limit']['ball']['saturation']
            self.segm_limits[0][0][2] = config_file['hsv_bottom_limit']['ball']['value']

            self.segm_limits[1][0][0] = config_file['hsv_bottom_limit']['allie_0']['hue']
            self.segm_limits[1][0][1] = config_file['hsv_bottom_limit']['allie_0']['saturation']
            self.segm_limits[1][0][2] = config_file['hsv_bottom_limit']['allie_0']['value']

            self.segm_limits[2][0][0] = config_file['hsv_bottom_limit']['allie_1']['hue']
            self.segm_limits[2][0][1] = config_file['hsv_bottom_limit']['allie_1']['saturation']
            self.segm_limits[2][0][2] = config_file['hsv_bottom_limit']['allie_1']['value']

            self.segm_limits[3][0][0] = config_file['hsv_bottom_limit']['allie_2']['hue']
            self.segm_limits[3][0][1] = config_file['hsv_bottom_limit']['allie_2']['saturation']
            self.segm_limits[3][0][2] = config_file['hsv_bottom_limit']['allie_2']['value']

            self.segm_limits[4][0][0] = config_file['hsv_bottom_limit']['allie_shirt']['hue']
            self.segm_limits[4][0][1] = config_file['hsv_bottom_limit']['allie_shirt']['saturation']
            self.segm_limits[4][0][2] = config_file['hsv_bottom_limit']['allie_shirt']['value']

            self.segm_limits[5][0][0] = config_file['hsv_bottom_limit']['enemy_shirt']['hue']
            self.segm_limits[5][0][1] = config_file['hsv_bottom_limit']['enemy_shirt']['saturation']
            self.segm_limits[5][0][2] = config_file['hsv_bottom_limit']['enemy_shirt']['value']

            # > Top limits
            self.segm_limits[0][1][0] = config_file['hsv_bottom_limit']['ball']['hue']
            self.segm_limits[0][1][1] = config_file['hsv_bottom_limit']['ball']['saturation']
            self.segm_limits[0][1][2] = config_file['hsv_bottom_limit']['ball']['value']

            self.segm_limits[1][1][0] = config_file['hsv_bottom_limit']['allie_0']['hue']
            self.segm_limits[1][1][1] = config_file['hsv_bottom_limit']['allie_0']['saturation']
            self.segm_limits[1][1][2] = config_file['hsv_bottom_limit']['allie_0']['value']

            self.segm_limits[2][1][0] = config_file['hsv_bottom_limit']['allie_1']['hue']
            self.segm_limits[2][1][1] = config_file['hsv_bottom_limit']['allie_1']['saturation']
            self.segm_limits[2][1][2] = config_file['hsv_bottom_limit']['allie_1']['value']

            self.segm_limits[3][1][0] = config_file['hsv_bottom_limit']['allie_2']['hue']
            self.segm_limits[3][1][1] = config_file['hsv_bottom_limit']['allie_2']['saturation']
            self.segm_limits[3][1][2] = config_file['hsv_bottom_limit']['allie_2']['value']

            self.segm_limits[4][1][0] = config_file['hsv_bottom_limit']['allie_shirt']['hue']
            self.segm_limits[4][1][1] = config_file['hsv_bottom_limit']['allie_shirt']['saturation']
            self.segm_limits[4][1][2] = config_file['hsv_bottom_limit']['allie_shirt']['value']

            self.segm_limits[5][1][0] = config_file['hsv_bottom_limit']['enemy_shirt']['hue']
            self.segm_limits[5][1][1] = config_file['hsv_bottom_limit']['enemy_shirt']['saturation']
            self.segm_limits[5][1][2] = config_file['hsv_bottom_limit']['enemy_shirt']['value']

    def process_frame(self, frame):
        self.frame = frame

        self.preparer.set_frame(self.frame)
        self.preparer.apply_all_filters()

        self.processed_frame = self.preparer.get_processed_frame()
        return self.processed_frame


    def get_frame_processed(self):
        return self.processed_frame
