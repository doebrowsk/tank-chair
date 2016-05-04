import matplotlib.pyplot as plt
import numpy as np

if __name__ == '__main__':
	# Data from cpp
	obstacles = [[0, 20, 30, 30], [0, 40, 10, 50], [0, 60, 30, 70], [20, 70, 30, 90], [10, 80, 20, 90], [20, 0, 90, 10], [80, 10, 90, 20], [40, 10, 50, 80], [20, 40, 40, 50], [50, 30, 90, 40], [60, 20, 70, 60], [80, 40, 90, 90], [60, 70, 70, 100]]
	x = []
	y = []
	links = []
	path = [[5, 5], [29.5, 19.5], [30.5, 20.5], [30.5, 29.5], [19.5, 40.5], [19.5, 49.5], [23.5, 53.5], [30.5, 60.5], [39.5, 79.5], [40.5, 80.5], [49.5, 80.5], [64.5, 65.5], [69.5, 69.5], [70.5, 70.5], [79.5, 89.5], [80.5, 90.5], [89.5, 90.5], [90.5, 89.5], [90.5, 30.5], [76.5, 16.5], [60.5, 19.5], [56.5, 23.5], [55.5, 25.5]]
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