#!/usr/bin/python
import sys, os

from construct import *


LogMessage = Struct("LogMessage",
    Magic("\xa3\x95\x86"),
    ULInt32("timestamp"),
    Array(8, ULInt16("ch"))
)


CYCLES=1
LOW_COUNT=1000
HIGH_COUNT=400
STEP=100

def write_signal(val):
	ch = [val]*8
	msg = LogMessage.build(Container(timestamp=0xccaaffee,ch=ch))
	#print repr(msg)
	os.write(1,bytes(msg))
	print >>sys.stderr, val, 

# calibrate full throttle
for high in range(5*HIGH_COUNT):
	write_signal(2000)


for cycle in range(CYCLES):
	for step in range(1500,2001,STEP):
		# 10%-> 50...100%
		print >>sys.stderr
		print >>sys.stderr
		for low in range(LOW_COUNT):
			write_signal(1100)

		print >>sys.stderr
		for high in range(HIGH_COUNT):
			write_signal(step)

		# 20%-> 50...100%
		print >>sys.stderr
		print >>sys.stderr
		for low in range(LOW_COUNT):
			write_signal(1200)

		print >>sys.stderr
		for high in range(HIGH_COUNT):
			write_signal(step)
	
		# 30%-> 50...100%
		print >>sys.stderr
		print >>sys.stderr
		for low in range(LOW_COUNT):
			write_signal(1300)

		print >>sys.stderr
		for high in range(HIGH_COUNT):
			write_signal(step)
	
	for step in range(1100,1701,STEP):
		# 10%...70% -> 100%
		print >>sys.stderr
		print >>sys.stderr
		for low in range(LOW_COUNT):
			write_signal(step)

		print >>sys.stderr
		for high in range(HIGH_COUNT):
			write_signal(2000)
