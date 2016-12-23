




def transObj(fileName, massCenter) :
	objFile = open(fileName, 'r')
	newObjFile = open("centered.obj", 'w')
	for line in objFile :
		if line[0] != 'v' or line[1] != ' ' :
			newObjFile.write(line)
			continue
		line = line[1:]
		line = line.strip()
		positions = line.split(' ')
		newline = "v"
		for i in range(3) :
			positions[i] = positions[i].strip()
			positions[i] = float(positions[i]) - massCenter[i]
			newline += (" " + str(positions[i]))
		newline += '\n'
		newObjFile.write(newline)
	objFile.close()
	newObjFile.close()

# transObj("MyPawn.obj", (-4.40814, 1.30989, 10.4309))
# transObj("car1.obj", (28.0153, 37.0629, 0.0238925))
transObj("cylinder.obj", (-4.67505, 1.5531, 82.9159))
