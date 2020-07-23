import queue
from numpy import var

q1 = queue.Queue(5)

for i in range(10):
    if q1.full():
        q1.get()

    q1.put(i)
    print q1.queue
    print var(q1.queue)
