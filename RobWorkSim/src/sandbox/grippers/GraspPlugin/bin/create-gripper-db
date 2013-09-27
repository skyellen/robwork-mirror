#!/usr/bin/python

import sys
import os
import math
import random
import argparse



class GripperGenerator:
	"Class for generating grippers."
	
	def __init__(self):
		self.gripperName = "gripper"
		self._gripperId = 0
		self.length = (0.01, 0.2)
		self.width = (0.001, 0.05)
		self.depth = (0.001, 0.05)
		self.chfChance = 0.5
		self.chfDepth = (0, 1)
		self.chfAngle = (15, 75)
		self.tcpOffset = (0.2, 0.8)
		self.cutChance = 0.67
		self.cutDepth = (0, 0.05)
		self.cutAngle = (60, 120)
		self.cutRadius = (0, 0.1)
		self.opening = (0.05, 0.1)
		self.jawdist = (0, 0.01)
		self.force = (25, 75)
		
	def generate(self, stl=False):
		# setup gripper name
		name = self.gripperName + str(self._gripperId)
		self._gripperId += 1
		
		# randomize general dimensions
		cutout = random.randint(0, 1)
		length = random.uniform(*self.length)
		width = random.uniform(*self.width)
		depth = random.uniform(*self.depth)
		
		chfDepth = 0
		chfAngle = 0
		if random.uniform(0, 1) < self.chfChance:
			chfDepth = random.uniform(*self.chfDepth)
			chfAngle = random.uniform(*self.chfAngle)
			
		tcpOffset = random.uniform(*self.tcpOffset)
		cutAngle = random.uniform(*self.cutAngle)
		opening = random.uniform(*self.opening)
		force = random.uniform(*self.force)
		
		# find tcp position
		tcp = tcpOffset * length
		
		# generate cut depth
		cutDepth = 0
		if random.uniform(0, 1) < self.cutChance:
			d = length - chfDepth * width * math.tan(math.radians(chfAngle))
			lwidth = width
			if tcp > d:
				lwidth = width - (tcp - d) * 1.0/math.tan(math.radians(chfAngle))
			cutDepth = random.uniform(self.cutDepth[0], min(self.cutDepth[1], lwidth-0.001))
		
		# generate cut radius
		cutRadius = random.uniform(max(self.cutRadius[0], cutDepth), self.cutRadius[1])
		
		# generate jawdist
		jawdist = random.uniform(self.jawdist[0], min(self.jawdist[1], opening))
		
		# make jaw parameters
		jawParameters = (cutout, length, width, depth, chfDepth, chfAngle, tcp, cutDepth, cutAngle, cutRadius)
		jawStr = "".join(str(p)+' ' for p in jawParameters)
		
		# construct command
		cd = "${RWSIM_ROOT}src/sandbox/grippers/GraspPlugin/bin/"
		cmd = cd+"create-gripper-xml --name {} -j {} --tcp {} --jawdist {} --opening {} --force {} {}"\
			.format(name, jawStr, tcp, jawdist, opening, force, ("", "--stl")[stl])
			
		# execute
		#print cmd
		os.system(cmd)

def main():
	parser = argparse.ArgumentParser(description="Generates gripper database.")
	parser.add_argument("number", nargs="?", default="1", help="number of grippers to generate")
	parser.add_argument("--stl", action="store_true", help="generate STL files")
	args = parser.parse_args()
	
	generator = GripperGenerator()
	
	loading = ['|', '/', '-', '\\']
	sys.stdout.write("Generating gripper database...  ")
	
	for i in range(0, int(args.number)):
		generator.generate(bool(args.stl))
		sys.stdout.write("\b"+loading[i%4])
		sys.stdout.flush()

	sys.stdout.write("\b Done.\n")

if __name__ == "__main__":
	main()
