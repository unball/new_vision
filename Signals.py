from gi.repository import Gtk
import mainWindow
import frameRenderer

class Signals:
	def __init__(self):
		self.pageSelected = 0
		
	def onDestroy(self, *args):
		Gtk.main_quit()
		
	def segmentarCampo_update_points(self, widget, event):
		if self.pageSelected == 0:
			frameRenderer.segmentarCampo().update_points([int(event.x), int(event.y)])
		
	def segmentarPreto_set_hmin(self, widget):
		frameRenderer.segmentarPreto().update_hsv_interval(int(widget.get_value()), 0)
		
	def segmentarPreto_set_smin(self, widget):
		frameRenderer.segmentarPreto().update_hsv_interval(int(widget.get_value()), 1)
		
	def segmentarPreto_set_vmin(self, widget):
		frameRenderer.segmentarPreto().update_hsv_interval(int(widget.get_value()), 2)
		
	def segmentarPreto_set_hmax(self, widget):
		frameRenderer.segmentarPreto().update_hsv_interval(int(widget.get_value()), 3)
		
	def segmentarPreto_set_smax(self, widget):
		frameRenderer.segmentarPreto().update_hsv_interval(int(widget.get_value()), 4)
		
	def segmentarPreto_set_vmax(self, widget):
		frameRenderer.segmentarPreto().update_hsv_interval(int(widget.get_value()), 5)
		
	def segmentarTime_set_hmin(self, widget):
		frameRenderer.segmentarTime().update_hsv_interval(int(widget.get_value()), 0)
		
	def segmentarTime_set_smin(self, widget):
		frameRenderer.segmentarTime().update_hsv_interval(int(widget.get_value()), 1)
		
	def segmentarTime_set_vmin(self, widget):
		frameRenderer.segmentarTime().update_hsv_interval(int(widget.get_value()), 2)
		
	def segmentarTime_set_hmax(self, widget):
		frameRenderer.segmentarTime().update_hsv_interval(int(widget.get_value()), 3)
		
	def segmentarTime_set_smax(self, widget):
		frameRenderer.segmentarTime().update_hsv_interval(int(widget.get_value()), 4)
		
	def segmentarTime_set_vmax(self, widget):
		frameRenderer.segmentarTime().update_hsv_interval(int(widget.get_value()), 5)
		
	def onConfigPageChange(self, page, widget, num):
		self.pageSelected = num
		if   self.pageSelected == 0:
			mainWindow.MainWindow().update_frame_thread.set_frame_renderer(frameRenderer.segmentarCampo())
		elif self.pageSelected == 1:
			mainWindow.MainWindow().update_frame_thread.set_frame_renderer(frameRenderer.segmentarPreto())
		elif self.pageSelected == 2:
			mainWindow.MainWindow().update_frame_thread.set_frame_renderer(frameRenderer.segmentarTime())
