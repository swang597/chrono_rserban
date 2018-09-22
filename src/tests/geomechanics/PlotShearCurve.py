import sys
import matplotlib.pyplot as plt

if len(sys.argv) != 2:
	print('usage: ' + sys.argv[0] + ' <data file>')
	exit(1)

t = []
x = []
fm = []
fc = []
ffm = []
area = []

shear_stress = []

# t,x,fm,fc,ffm,A,iter
file = open(sys.argv[1])
for line in file.readlines()[1:]:
	tok = line.split(',')
	t.append(float(tok[0]))
	x.append(float(tok[1]))
	fm.append(float(tok[2]))
	fc.append(float(tok[3]))
	ffm.append(float(tok[4]))
	area.append(float(tok[5]))

for i in range(len(ffm)):
	shear_stress.append(ffm[i] / area[i])

plt.subplot(211)
plt.plot(x, ffm, 'k-')
plt.title('Displacement-Shear Force (Filtered) Curve')
plt.xlabel('Shear Displacement (m)')
plt.ylabel('Shear Force Filtered (motor) (N)')

plt.subplot(212)
plt.plot(x, shear_stress, 'b-')
plt.title('Displacement-Shear Stess Curve')
plt.xlabel('Shear Displacement (m)')
plt.ylabel('Shear Stress (motor) (N/m2)')

# plt.figure(1)
# plt.subplot(311)
# plt.plot(x, fm, 'r-')
# plt.title('Displacement-Shear Force Curve')
# plt.xlabel('Shear Displacement (m)')
# plt.ylabel('Shear Force (motor) (N)')

# plt.subplot(312)
# plt.plot(x, fc, 'g-')
# plt.title('Displacement-Shear Curve')
# plt.xlabel('Shear Displacement (m)')
# plt.ylabel('Shear Force (contact) (N)')

# plt.subplot(313)
# plt.plot(t, x, 'b-')
# plt.title('Time-Displacement Curve')
# plt.xlabel('Time (t)')
# plt.ylabel('Shear Displacement (m)')

plt.show()