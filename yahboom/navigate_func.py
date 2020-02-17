import multiprocessing
import time
import wander_func
import scan_func

if __name__ == '__main__':
    
    p1 = multiprocessing.Process(target=wander_func.wander)
    p2 = multiprocessing.Process(target=scan_func.scan)

    p1.start()
    p2.start()

    time.sleep(30)

    p1.terminate()
    p2.terminate()

