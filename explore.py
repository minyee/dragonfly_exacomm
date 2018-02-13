import os

end = False
fileNameList = os.listdir()
for fileName in fileNameList:
	[name, suffix] = fileName.split(".")
	if suffix != ".txt":
		continue
	

