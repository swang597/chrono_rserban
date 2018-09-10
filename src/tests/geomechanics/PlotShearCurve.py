import sys
import matplotlib.pyplot as plt

if len(sys.argv) != 2:
	print('usage: ' + sys.argv[0] + ' <data file>')
	exit(1)
	
x = []
f = []

# t,x,f
file = open(sys.argv[1])
for line in file.readlines()[1:]:
	tok = line.split(',')
	x.append(float(tok[1]))
	f.append(float(tok[2]))

plt.plot(x, f, 'r-')
plt.xlabel('Shear Displacement (m)')
plt.ylabel('Shear Force (N)')
plt.show()