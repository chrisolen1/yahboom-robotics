import multiprocessing
import time

if __name__ == '__main__':
    
    p1 = multiprocessing.Process(target=lambda: __import__("wander"))
    p2 = multiprocessing.Process(target=lambda: __import__("scan"))

    p1.start()
    p2.start()

    time.sleep(10)

    p1.terminate()
    p2.terminate()
    

