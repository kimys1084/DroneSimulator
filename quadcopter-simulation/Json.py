

def ListToJsonFile(list, filename):

	listLength = len(list)
	trash = 0.0
	f = open(filename, "w")
	f.write("{\n")
	f.write("\"index\" : [")
	for idx in range(listLength):
		f.write("{\n")
		x,y,z = list[idx].pos
		rotx, roty, rotz = list[idx].attitude
		
	
		data = "	\"t\": %f,\n" %trash
		f.write(data)
		data = "	\"x\": %f,\n" %x
		f.write(data)
		data = "	\"y\": %f,\n" %y
		f.write(data)
		data = "	\"z\": %f,\n" %z
		f.write(data)
		data = "	\"Rotx\": %f,\n" %rotx
		f.write(data)
		data = "	\"Roty\": %f,\n" %roty
		f.write(data)
		data = "	\"Rotz\": %f\n" %rotz
		f.write(data)

		if idx == len(list)-1:
			f.write("	}]\n")
		else:
			f.write("	},\n")
	f.write("\n}")
	f.close()