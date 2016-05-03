import matplotlib.pyplot as plt
import numpy as np

if __name__ == '__main__':
	# Data from cpp
	obstacles = [[2, 0, 3, 99], [3, 98, 97, 99], [97, 0, 98, 99]]
	x = []
	y = []
	links = []
	path = [[1, 1], [0.247936, 99.5224], [93.317, 99.8055], [98.2358, 98.9578], [99, 1]]
	# Map
	x_fill = np.arange(0.0, 100, 0.01)
	for (x1,y1,x2,y2) in obstacles:
		plt.fill_between(x_fill, y1, y2, where=[(x1<=a<=x2) for a in x_fill], facecolor='gray')
		
	# Points
	plt.plot(x, y, 'b.')
	
	# Links
	for (x1,y1,x2,y2) in links:
		plt.plot([x1, x2], [y1, y2], color='green', linestyle='-', linewidth=1)
		
	# Path
	prevP = None;
	for p in path:
		if prevP != None:
			plt.plot([prevP[0], p[0]], [prevP[1], p[1]], color='red', linestyle='-', linewidth=2)
		prevP=p
		
	# Show the plot
	plt.show()