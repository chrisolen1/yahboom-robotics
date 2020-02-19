import multiprocessing
import time
import wander_func
import scan_func


    
p1 = multiprocessing.Process(target=wander_func.wander, kwargs={'day_mode':True})
p2 = multiprocessing.Process(target=scan_func.scan)

p1.start()
p2.start()

time.sleep(30)

p1.terminate()
p2.terminate()

