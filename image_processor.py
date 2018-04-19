class ImageProcessor():
    def __init__(self, frame):
        self.frame = frame

    def set_frame(self, frame):
        self.frame = frame

    def apply_all_filters(self):
        geometrical_transformer()
        blur()
        hsv_conversor()
        

    def geometrical_transformer(self, points):
        pass

    def blur(self):
        pass

    def hsv_conversor(self):
        pass

    def get_frame_processed(self):
        return self.frame
