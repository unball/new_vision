from os import listdir
import singleton
from gi.repository import Gtk

class uiCamerasList(metaclass=singleton.Singleton):
	def __init__(self):
		self.cameras = set()
	
	def getCameras(self):
		return set(enumerate(sorted([c for c in listdir("/sys/class/video4linux/")])))

	def updateCameras(self, widget_list):
		for camera in sorted([c for c in self.getCameras().difference(self.cameras)]):
			self.cameras.add(camera)
			row = Gtk.ListBoxRow()
			row.index = camera[0]
			row.add(Gtk.Label(camera[1]))
			row.set_size_request(150,30)
			widget_list.insert(row,-1)
			widget_list.show_all()
