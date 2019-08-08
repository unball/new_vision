#!/usr/bin/env python3

import gi
gi.require_version('Gtk', '3.0')

import mainWindow

def main():
	w = mainWindow.MainWindow()
	w.run()

if __name__ == "__main__":
	main()
