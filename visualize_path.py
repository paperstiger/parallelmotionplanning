from klampt import *
from klampt import vis
import time
path = []
with open('path.txt') as lines:
	for line in lines:
	    path.append([float(v) for v in line.split()])

world = WorldModel()

res = world.readFile("klampt_data/tx90obstacles.xml")
if not res:
	raise RuntimeError("unable to load model")
robot = world.robot(0)
vis.add("world",world)
for i in range(len(path)):
	robot.setConfig([0]+path[i])
	print(path[i])
	vis.add("ghost" + str(i), [0]+path[i])
vis.show()
while vis.shown():
	time.sleep(0.1)

vis.kill()
