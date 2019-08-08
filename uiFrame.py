from gi.repository import GdkPixbuf

class uiFrame:
	
	def __init__(self, gtk_frame):
		self.gtk_frame = gtk_frame
	
	def do_update_frame(self, args):
		image_data, width, height, depth = args
		pixbuf = GdkPixbuf.Pixbuf.new_from_data(image_data.tostring(), GdkPixbuf.Colorspace.RGB, False, 8, width, height, depth*width)
		self.gtk_frame.set_from_pixbuf(pixbuf.copy())
