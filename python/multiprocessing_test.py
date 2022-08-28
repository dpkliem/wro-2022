
import multiprocessing
import time
 
def task():
    print('Sleeping for 0.5 seconds')
    time.sleep(0.5)
    print('Finished sleeping')
 
if __name__ == "__main__":
    start_time = time.perf_counter()
 
    # Creates two processes
    p1 = multiprocessing.Process(target=task)
    p2 = multiprocessing.Process(target=task)
 
    # Starts both processes
    p1.start()
    p2.start()
    p1.join()
    p2.join()
 
    finish_time = time.perf_counter()
 
    print(f"Program finished in {finish_time-start_time} seconds")