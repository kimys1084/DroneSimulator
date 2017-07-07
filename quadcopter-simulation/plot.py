import numpy as np
import matplotlib.pyplot as plt


def plot_err(desired, actual):

	desiredLineX = []
	desiredLineY = []
	desiredLineZ = []

	actualLineX = []
	actualLineY = []
	actualLineZ = []

	trajectoryLength = len(desired)
	for idx in range(trajectoryLength):
		x,y,z = desired[idx].pos
		desiredLineX.append(x)
		desiredLineY.append(y)
		desiredLineZ.append(z)

		x,y,z = actual[idx].pos
		actualLineX.append(x)
		actualLineY.append(y)
		actualLineZ.append(z)


	desiredLineX = np.array(desiredLineX)
	actualLineX = np.array(actualLineX)
	time = np.arange(0, 0.005*len(desiredLineX), 0.005)
	plt.plot(time, desiredLineZ, 'r--',time, actualLineZ, 'b--')
	plt.show()
