"""Laptop offline entrance-constrained A with the preserved mask K gate."""
from junction_k_mask import KMaskPath
from road_segments_v32 import RoadPath
class KEntrancePath(KMaskPath):
 def __init__(self,row=.6):
  super().__init__(row)
  self.base=RoadPath('road-entrance-v3.2',None,row)
