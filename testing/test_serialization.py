#needed to import GEMstack from top level directory
import sys
import os
sys.path.append(os.getcwd())

from GEMstack.utils import serialization
from GEMstack.state import PhysicalObject,ObjectPose,ObjectFrameEnum

def test_serialization():
	o = PhysicalObject(pose=ObjectPose(frame=ObjectFrameEnum.GLOBAL,t=0.0,x=2.0,y=4.0,yaw=0.5),
					   dimensions=(0.1,0.1,0.1),
					   outline=None)
	print(serialization.serialize(o))
	o2 = serialization.deserialize(serialization.serialize(o))
	assert o == o2

	coll = {'objects':[o]*5,'others':'hi'}
	print(serialization.serialize_collection(coll))
	coll2 = serialization.deserialize_collection(serialization.serialize_collection(coll))
	assert coll == coll2

if __name__=='__main__':
	test_serialization()