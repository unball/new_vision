import json

def getConfig():
	try:
		with open("config.json", "r") as f:
			return json.load(f)
	except:
		saveConfig({})
		return {}

def saveConfig(config):
	with open("config.json", "w") as f:
		json.dump(config, f)
