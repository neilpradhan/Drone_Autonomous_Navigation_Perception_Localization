#!/usr/bin/env python
import json
import matplotlib.pyplot as plt
import numpy as np
import math
from occupation_grid import *

from multiprocessing import Queue
import threading 

q = Queue()


for i in range(5):
	q.put(i)
while not q.empty():
    print(q.get())	