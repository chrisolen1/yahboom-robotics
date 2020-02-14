import multiprocessing

if __name__ == '__main__':
    
    p1 = multiprocessing.Process(target=lambda: __import__('record'))
    p2 = multiprocessing.Process(target=lambda: __import__('navigate'))
    p1.start()
    p2.start()
    p1.join()
    p2.join()
