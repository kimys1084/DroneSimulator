

def ListToJsonFile(list, filename):
	f = open(filename, "w")
	f.write("{\n")
	f.write("\"index\" : [")
	for idx in list:
		f.write("{\n")
		x,y,z = list[idx].pos
		rotx, roty, rotz = list[idx].attitude
		
		f.write("\"t\": ", 0,",")
		f.write("\"x\": ",x,",")
		f.write("\"y\": ",y,",")
		f.write("\"z\": ",z,",")
		f.write("\"Rotx\": ",rotx,",")
		f.write("\"Roty\": ",roty,",")
		f.write("\"Rotz\": ",rotz,"\n")

		if idx == len(list)-1:
			f.write("}]\n")
		else:
			f.write("},\n")
	f.write("\n}")
	f.close()