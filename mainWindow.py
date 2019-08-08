from threading import Thread, Event
from gi.repository import Gtk
import Signals
import uiFrame
import frameUpdater
import singleton

class MainWindow(metaclass=singleton.Singleton):

	def __init__(self):
		self.builder = None
		self.update_frame_thread = None
	
	def loadBuilder(self):
		if self.builder is None:
			self.builder = Gtk.Builder()
			self.builder.add_from_file("ui.glade")
	
	def getObject(self, name):
		self.loadBuilder()
		return self.builder.get_object(name)
	
	def run(self):
		# Load static UI
		self.loadBuilder()
		
		# Connect signals
		self.builder.connect_signals(Signals.Signals())
		
		# Show window
		window = self.getObject("window1")
		window.show_all()
		
		# Create webcam thread that updates UI frame
		ui_frame = uiFrame.uiFrame(self.getObject("frame"))
		self.update_frame_thread = frameUpdater.frameUpdater(ui_frame)
		self.update_frame_thread.run()
		
		# Start GTK main loop
		Gtk.main()
		
		# Stops threads
		self.update_frame_thread.stop()
		
