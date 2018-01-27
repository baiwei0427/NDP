import sys

def usage():
	print 'python %s input_file' % sys.argv[0]

if len(sys.argv) != 2:
	usage()
	sys.exit(1)

file = open(sys.argv[1])
lines = file.readlines()
file.close()

overall_fct = []
small_fct = []
median_fct = []
large_fct = []

for line in lines:
	if 'finished' not in line:
		continue

	words = line.split()
	#print words
	if len(words) != 9:
		continue

	size = int(words[-1])
	fct = float(words[-3])

	overall_fct.append(fct)
	
	if size <= 100 * 1024:
		small_fct.append(fct)
	elif size <= 1024 * 1024:
		median_fct.append(fct)
	else:
		large_fct.append(fct)

small_fct.sort()

print '%d flows in total' % len(overall_fct)
print 'Overall average FCT: %f ms' % (sum(overall_fct) / len(overall_fct))

print '%d small flows in (0, 100KB]' % len(small_fct)
print 'Small flows average FCT: %f ms' % (sum(small_fct) / len(small_fct))
print 'Small flows 99th FCT: %f ms' % small_fct[int(len(small_fct) * 0.99)]

print '%d median flows in (100KB, 1MB]' % len(median_fct)
print 'Median flows average FCT: %f ms' % (sum(median_fct) / len(median_fct))

print '%d median flows in (1MB, )' % len(large_fct)
print 'Large flows average FCT: %f ms' % (sum(large_fct) / len(large_fct))
