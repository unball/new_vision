#import rospy
#from std_msgs.msg import String
import mainWindow
from gi.repository import Gtk, Gdk
import singleton

class gameCommands(metaclass=singleton.Singleton):
	def __init__(self):
		self.goalAlly = 0
		self.goalEnemy = 0
		self.oponente = "Oponente"
		#rospy.init_node('game_commands')
		#pub = rospy.Publisher('game_commands', String, queue_size=1)
		#rate = rospy.Rate(30)
		
	def left(self):
		print ("L")
		#pub.publish("L")
		#rate.sleep()
	
	def right(self):
		print ("R")
		#pub.publish("R")
		#rate.sleep()
	
	def ally(self):
		estadoLabel = mainWindow.MainWindow().getObject("gameCommands_estado")
		placarLabel = mainWindow.MainWindow().getObject("gameCommands_placar")
		playPouseButton = mainWindow.MainWindow().getObject("gameCommands_playpause")
		
		if estadoLabel.get_text() == "O jogo está rodando":
			print("Goal")
			self.goalAlly += 1
			placarLabel.set_text("Placar\nUnBall " + str(self.goalAlly) + " x " + str(self.goalEnemy) + " " + self.oponente)
			estadoLabel.set_text("O jogo está pausado")
			estadoLabel.modify_fg(Gtk.StateFlags.NORMAL, Gdk.color_parse("red"))
			playPouseButton.set_label("Play")
			playPouseButton.modify_bg(Gtk.StateFlags.NORMAL, Gdk.color_parse("green"))
			#pub.publish("P")
			#rate.sleep()
	
	def enemy(self):
		estadoLabel = mainWindow.MainWindow().getObject("gameCommands_estado")
		placarLabel = mainWindow.MainWindow().getObject("gameCommands_placar")
		playPouseButton = mainWindow.MainWindow().getObject("gameCommands_playpause")
		
		if estadoLabel.get_text() == "O jogo está rodando":
			print("E Goal")
			self.goalEnemy += 1
			placarLabel.set_text("Placar\nUnBall " + str(self.goalAlly) + " x " + str(self.goalEnemy) + " " + self.oponente)
			estadoLabel.set_text("O jogo está pausado")
			estadoLabel.modify_fg(Gtk.StateFlags.NORMAL, Gdk.color_parse("red"))
			playPouseButton.set_label("Play")
			playPouseButton.modify_bg(Gtk.StateFlags.NORMAL, Gdk.color_parse("green"))
			#pub.publish("P")
			#rate.sleep()

	def fimdejogo(self, event):
		print ("Q")
	
	def IniciodeJogo(self):
		estadoLabel = mainWindow.MainWindow().getObject("gameCommands_estado")
		playPouseButton = mainWindow.MainWindow().getObject("gameCommands_playpause")
		if estadoLabel.get_text() == "Equipe UnBall":
			estadoLabel.set_text("O jogo está rodando")
			estadoLabel.modify_fg(Gtk.StateFlags.NORMAL, Gdk.color_parse("blue"))
			playPouseButton.set_label("Pause")
			playPouseButton.modify_bg(Gtk.StateFlags.NORMAL, Gdk.color_parse("red"))
			#pub.publish("G")
			#rate.sleep()
			
	def mudarTexto(self):
		estadoLabel = mainWindow.MainWindow().getObject("gameCommands_estado")
		playPouseButton = mainWindow.MainWindow().getObject("gameCommands_playpause")
		if estadoLabel.get_text() == "O jogo está rodando":
			estadoLabel.set_text("O jogo está pausado")
			estadoLabel.modify_fg(Gtk.StateFlags.NORMAL, Gdk.color_parse("red"))
			playPouseButton.set_label("Play")
			playPouseButton.modify_bg(Gtk.StateFlags.NORMAL, Gdk.color_parse("green"))
			#pub.publish("P")
			#rate.sleep()
		else:
			estadoLabel.set_text("O jogo está rodando")
			estadoLabel.modify_fg(Gtk.StateFlags.NORMAL, Gdk.color_parse("blue"))
			playPouseButton.set_label("Pause")
			playPouseButton.modify_bg(Gtk.StateFlags.NORMAL, Gdk.color_parse("red"))
			#pub.publish("G")
			#rate.sleep()

