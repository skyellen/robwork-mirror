#!/usr/bin/python

import os
import math
import random



class GripperGenerator:
	"Class for generating grippers."
	
	def __init__(self):
		self.gripperName = "gripper"
		self._gripperId = 0
		self.length = (0.01, 0.2)
		self.width = (0.001, 0.05)
		self.depth = (0.001, 0.05)
		self.chfDepth = (0, 1)
		self.chfAngle = (0, 90)
		self.tcpOffset = (0, 1)
		self.cutDepth = (0, 0.05)
		self.cutAngle = (60, 120)
		self.cutRadius = (0, 0.1)
		self.opening = (0.05, 0.1)
		self.jawdist = (0, 0.01)
		self.force = (25, 75)
		
	def generate(self, stl=False):
		# setup gripper name
		name = self.gripperName + str(self._gripperId)
		
		# randomize general dimensions
		cutout = random.randint(0, 1)
		length = random.uniform(*self.length)
		width = random.uniform(*self.width)
		depth = random.uniform(*self.depth)
		chfDepth = random.uniform(*self.chfDepth)
		chfAngle = random.uniform(*self.chfAngle)
		tcpOffset = random.uniform(*self.tcpOffset)
		cutAngle = random.uniform(*self.cutAngle)
		cutRadius = random.uniform(*self.cutRadius)
		opening = random.uniform(*self.opening)
		force = random.uniform(*self.force)
		
		# find tcp position
		tcp = tcpOffset * length
		
		# generate cut depth
		d = length - chfDepth * width * math.tan(math.radians(chfAngle))
		lwidth = width
		if tcp > d:
			lwidth = width - (tcp - d) * 1.0/math.tan(math.radians(chfAngle))
		cutDepth = random.uniform(self.cutDepth[0], min(self.cutDepth[1], lwidth+0.001))
		
		# generate jawdist
		jawdist = random.uniform(self.jawdist[0], min(self.jawdist[1], opening))
		
		# make jaw parameters
		jawParameters = (cutout, length, width, depth, chfDepth, chfAngle, tcp, cutDepth, cutAngle, cutRadius)
		jawStr = "".join(str(p)+' ' for p in jawParameters)
		
		# construct command
		cd = "/home/dagothar/robwork/trunk/RobWorkSim/src/sandbox/grippers/GraspPlugin/bin/"
		cmd = cd+"create-gripper-xml --name {0} -j {1} --jawdist {2} --opening {3} --force {4} {5}"\
			.format(name, jawStr, jawdist, opening, force, ("", "--stl")[stl])
			
		# execute
		#print cmd
		os.system(cmd)

def main():
	generator = GripperGenerator()
	generator.generate(True)


if __name__ == "__main__":
	main()
