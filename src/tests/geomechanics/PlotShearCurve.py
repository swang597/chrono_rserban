import sys
import matplotlib.pyplot as plt

if len(sys.argv) != 2:
	print('usage: ' + sys.argv[0] + ' <data file>')
	exit(1)

t = []
x = []
f = []

# t,x,f
file = open(sys.argv[1])
for line in file.readlines()[1:]:
	tok = line.split(',')
	t.append(float(tok[0]))
	x.append(float(tok[1]))
	f.append(float(tok[2]))

# plt.plot(t, x, 'b-')
plt.plot(x, f, 'r-')
plt.xlabel('Shear Displacement (m)')
plt.ylabel('Shear Force (N)')
plt.show()