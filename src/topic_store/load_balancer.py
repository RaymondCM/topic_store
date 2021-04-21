#!/usr/bin/env python

#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

# LoadBalancing utilities from raytils (https://github.com/RaymondKirk/raytils, https://pypi.org/project/raytils/)
import os
import re
import subprocess
import uuid

try:
    import queue
except ImportError:
    import Queue as queue

import threading

from collections import deque
from timeit import default_timer as timer
from time import time as _time
try:
    import threading as _threading
except ImportError:
    import dummy_threading as _threading


class FPSCounter:
    NOT_INITIALIZED_VALUE = -1

    def __init__(self, queue_length=30):
        self.__frame_times = deque(maxlen=int(queue_length))
        self.__start = None

    def reset(self):
        """Clears the timer history"""
        self.__frame_times.clear()

    def start(self):
        """Starts the timer"""
        self.__start = timer()

    def tic(self):
        """Starts the timer"""
        self.start()

    def stop(self):
        """Ends the timer and adds elapsed time since start() or tic() called to history"""
        if not self.__start:
            return
        self.__frame_times.append(timer() - self.__start)
        self.__start = None

    def toc(self):
        """Ends the timer and adds elapsed time since start() or tic() called to history"""
        self.stop()

    def get_fps(self):
        """Returns the FPS"""
        time_sum = sum(self.__frame_times)
        return (len(self.__frame_times) / time_sum) if time_sum else self.NOT_INITIALIZED_VALUE

    def get_times(self):
        """Returns the total time, ms and FPS"""
        if len(self.__frame_times) == 0:
            return self.NOT_INITIALIZED_VALUE, self.NOT_INITIALIZED_VALUE, self.NOT_INITIALIZED_VALUE
        total = sum(self.__frame_times)
        time_avg = sum(self.__frame_times) / len(self.__frame_times)
        ms_avg, fps = time_avg * 1000, 1. / time_avg
        return total, ms_avg, fps

    def __str__(self):
        return "Total Time: {:.2f}s Avg Time: {:.2f}ms ({:.2f} fps)".format(*self.get_times())


class Worker(threading.Thread):
    _exit_job = uuid.uuid4()  # special id to terminate thread
    """Runs jobs from LoadBalancer.
    Work from the LoadBalancer should be in the form of [func, args, kwargs].
    func should take a parameter job_meta containing specifics about the worker running the job.
    """
    def __init__(self, q, worker_id, *args, **kwargs):
        self.q = q
        self.worker_id = worker_id
        self.data_retrieval_rate = FPSCounter(q.maxsize)
        self.processing_rate = FPSCounter(q.maxsize)
        super(Worker, self).__init__(*args, **kwargs)

    def run(self):
        while True:
            try:
                work = self.q.get()
                # q.get will wait indefinitely and hang the main thread when exiting
                #  so allow a unique string to terminate the thread
                if isinstance(work, uuid.UUID) and work == Worker._exit_job:
                    self.q.task_done()
                    return
            except queue.Empty:
                return

            # Create dictionary of meta values to pass to the worker
            job_meta = {
                "worker_id": self.worker_id,  # This is my id
                "worker_job_processing_rate": self.processing_rate.get_fps(),  # I process data every n times per second
            }

            self.processing_rate.tic()
            work[0](*work[1], job_meta=job_meta, **work[2])
            self.processing_rate.toc()
            self.q.task_done()


class Queue(object):
    """Custom implementation of queue for python2.7 compatibility """
    def __init__(self, maxsize=0):
        self.maxsize = maxsize
        self._init(maxsize)
        # mutex must be held whenever the queue is mutating.  All methods
        # that acquire mutex must release it before returning.  mutex
        # is shared between the three conditions, so acquiring and
        # releasing the conditions also acquires and releases mutex.
        self.mutex = _threading.Lock()
        # Notify not_empty whenever an item is added to the queue; a
        # thread waiting to get is notified then.
        self.not_empty = _threading.Condition(self.mutex)
        # Notify not_full whenever an item is removed from the queue;
        # a thread waiting to put is notified then.
        self.not_full = _threading.Condition(self.mutex)
        # Notify all_tasks_done whenever the number of unfinished tasks
        # drops to zero; thread waiting to join() is notified to resume
        self.all_tasks_done = _threading.Condition(self.mutex)
        self.unfinished_tasks = 0

    def task_done(self):
        """Indicate that a formerly enqueued task is complete.

        Used by Queue consumer threads.  For each get() used to fetch a task,
        a subsequent call to task_done() tells the queue that the processing
        on the task is complete.

        If a join() is currently blocking, it will resume when all items
        have been processed (meaning that a task_done() call was received
        for every item that had been put() into the queue).

        Raises a ValueError if called more times than there were items
        placed in the queue.
        """
        self.all_tasks_done.acquire()
        try:
            unfinished = self.unfinished_tasks - 1
            if unfinished <= 0:
                if unfinished < 0:
                    raise ValueError('task_done() called too many times')
                self.all_tasks_done.notify_all()
            self.unfinished_tasks = unfinished
        finally:
            self.all_tasks_done.release()

    def join(self):
        """Blocks until all items in the Queue have been gotten and processed.

        The count of unfinished tasks goes up whenever an item is added to the
        queue. The count goes down whenever a consumer thread calls task_done()
        to indicate the item was retrieved and all work on it is complete.

        When the count of unfinished tasks drops to zero, join() unblocks.
        """
        self.all_tasks_done.acquire()
        try:
            while self.unfinished_tasks:
                self.all_tasks_done.wait()
        finally:
            self.all_tasks_done.release()

    def qsize(self):
        """Return the approximate size of the queue (not reliable!)."""
        self.mutex.acquire()
        n = self._qsize()
        self.mutex.release()
        return n

    def empty(self):
        """Return True if the queue is empty, False otherwise (not reliable!)."""
        self.mutex.acquire()
        n = not self._qsize()
        self.mutex.release()
        return n

    def full(self):
        """Return True if the queue is full, False otherwise (not reliable!)."""
        self.mutex.acquire()
        n = 0 < self.maxsize == self._qsize()
        self.mutex.release()
        return n

    def put(self, item, block=True, timeout=None):
        """Put an item into the queue.

        If optional args 'block' is true and 'timeout' is None (the default),
        block if necessary until a free slot is available. If 'timeout' is
        a non-negative number, it blocks at most 'timeout' seconds and raises
        the Full exception if no free slot was available within that time.
        Otherwise ('block' is false), put an item on the queue if a free slot
        is immediately available, else raise the Full exception ('timeout'
        is ignored in that case).
        """
        self.not_full.acquire()
        try:
            if self.maxsize > 0:
                if not block:
                    if self._qsize() == self.maxsize:
                        raise queue.Full
                elif timeout is None:
                    while self._qsize() == self.maxsize:
                        self.not_full.wait()
                elif timeout < 0:
                    raise ValueError("'timeout' must be a non-negative number")
                else:
                    endtime = _time() + timeout
                    while self._qsize() == self.maxsize:
                        remaining = endtime - _time()
                        if remaining <= 0.0:
                            raise queue.Full
                        self.not_full.wait(remaining)
            self._put(item)
            self.unfinished_tasks += 1
            self.not_empty.notify()
        finally:
            self.not_full.release()

    def put_nowait(self, item):
        """Put an item into the queue without blocking.

        Only enqueue the item if a free slot is immediately available.
        Otherwise raise the Full exception.
        """
        return self.put(item, False)

    def get(self, block=True, timeout=None):
        """Remove and return an item from the queue.

        If optional args 'block' is true and 'timeout' is None (the default),
        block if necessary until an item is available. If 'timeout' is
        a non-negative number, it blocks at most 'timeout' seconds and raises
        the Empty exception if no item was available within that time.
        Otherwise ('block' is false), return an item if one is immediately
        available, else raise the Empty exception ('timeout' is ignored
        in that case).
        """
        self.not_empty.acquire()
        try:
            if not block:
                if not self._qsize():
                    raise queue.Empty
            elif timeout is None:
                while not self._qsize():
                    self.not_empty.wait()
            elif timeout < 0:
                raise ValueError("'timeout' must be a non-negative number")
            else:
                endtime = _time() + timeout
                while not self._qsize():
                    remaining = endtime - _time()
                    if remaining <= 0.0:
                        raise queue.Empty
                    self.not_empty.wait(remaining)
            item = self._get()
            self.not_full.notify()
            return item
        finally:
            self.not_empty.release()

    def get_nowait(self):
        """Remove and return an item from the queue without blocking.

        Only get an item if one is immediately available. Otherwise
        raise the Empty exception.
        """
        return self.get(False)

    # Override these methods to implement other queue organizations
    # (e.g. stack or priority queue).
    # These will only be called with appropriate locks held

    # Initialize the queue representation
    def _init(self, maxsize):
        self.queue = deque()

    def _qsize(self, len=len):
        return len(self.queue)

    # Put a new item in the queue
    def _put(self, item):
        self.queue.append(item)

    # Get an item from the queue
    def _get(self):
        return self.queue.popleft()


class LoadBalancer(Queue):
    """Add jobs in the form of (func, args, kwargs) to a task queue and automatically scales number of workers.
    Usage:
    .. code-block:: python
        from raytils.system import LoadBalancer
        # We will add heavy tasks to the worker to not block the main thread
        workers = LoadBalancer(maxsize=30, threads=8)
        # Your function must have a job_meta keyword argument so your workers
        # can pass important information about themselves
        def save_dict(file_path, data, job_meta):
            import json
            with open(file_path, "w") as fh:
                json.dump(data, fh)
        # Save 10000 dictionaries to files to demonstrate a heavy load
        for idx in range(10000):
            dict_to_save = {i: i for i in range(10000)}
            file_path = f"{idx}.json"
            # Serially we would do save_dict(file_path, dict_to_save)
            # However we can do it much faster
            workers.add_task(save_dict, [file_path, dict_to_save])
    """
    def __init__(self, maxsize=0, threads=1, auto=True):
        """
        Example:
        Args:
            maxsize: Maximum size of the created jobs queue. If maxsize is <= 0, the queue size is infinite.
            threads: Number of workers to start threads for
            auto: If true workers will be automatically created if the demand is too high.
        """
        super(LoadBalancer, self).__init__(maxsize=maxsize)
        self.max_threads = self._available_cpu_count()
        self.auto = auto
        self.number_of_threads = 0
        self.tasks_per_second = FPSCounter(queue_length=maxsize)
        # Number of seconds to allow queue to catch up before adding more cores
        self._thread_grace_period = 2.0  # TODO: these hard-coded values should be tunable
        self._last_thread_added = timer()
        self._add_thread_threshold_percent = 0.8  # TODO: these hard-coded values should be tunable
        self._workers = {}  # id to worker lookup
        for _ in range(threads):
            self._add_thread(check_grace_period=False)

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.join()

    def join(self):
        # Add N dummy jobs so all workers grab one final task then exit
        for _ in range(len(self._workers)):
            self.put_nowait(Worker._exit_job)
        super(LoadBalancer, self).join()

    def _add_thread(self, check_grace_period=True):
        """Function that's used internally. If the queue of jobs ever becomes full a new thread is spawned to try to
        handle the new information in realtime"""
        # Do not add more threads if the CPU cannot support it
        if self.number_of_threads >= self.max_threads:
            return

        # Grace period so cores don't quickly accelerate to system maximum
        if check_grace_period and (timer() - self._last_thread_added) <= self._thread_grace_period:
            return

        # Keep reference to thread
        worker_id = self.number_of_threads
        self._workers[worker_id] = Worker(self, worker_id=worker_id)
        self._workers[worker_id].start()
        self._last_thread_added = timer()

        # Increment ID for next call to add thread
        self.number_of_threads += 1

    def add_task(self, func, args=None, kwargs=None, wait=False):
        """Add a task to be processed by one of the available workers
        Args:
            func: callable of the signature func(*args, **kwargs) or func(arg1, arg2, ..., kwarg1, job_meta=None)
            args: list of arguments to pass to the func calling func(*args, job_meta={...}, **kwargs)
            kwargs: list of keyword arguments to pass to the func calling func(*args, job_meta={...}, **kwargs)
            wait: If true blocks until the item is placed in the queue
        Returns:
            bool: True if item placed in the queue otherwise false
        """
        # If the number of tasks_per_second being added is lower than the current approx queue size
        # then add a thread to deal with the extra work
        if args is None:
            args = []
        if kwargs is None:
            kwargs = {}
        self.tasks_per_second.toc()
        self.tasks_per_second.tic()

        # Add a thread if queue_size growing faster than current processing or is nearing capacity
        if self.auto:
            queue_size = self.qsize()
            queue_capacity = queue_size / self.maxsize
            if queue_size >= self.tasks_per_second.get_fps() or queue_capacity >= self._add_thread_threshold_percent:
                self._add_thread()

        try:
            if wait:
                self.put((func, args, kwargs))
            else:
                self.put_nowait((func, args, kwargs))
        except queue.Full:
            if self.auto:
                # If the queue becomes full add a thread to balance the workload
                self._add_thread()  # attempt to add a thread to balance the workload
            return False

        return True

    @staticmethod
    def _available_cpu_count():
        """ Number of available virtual or physical CPUs on this system, i.e. user/real as output by time(1)

        Inspired by: https://stackoverflow.com/a/1006301
        """

        # cpuset may restrict the number of *available* processors
        try:
            with open('/proc/self/status') as fh:
                m = re.search(r'(?m)^Cpus_allowed:\s*(.*)$', fh.read())
            if m:
                res = bin(int(m.group(1).replace(',', ''), 16)).count('1')
                if res > 0:
                    return res
        except IOError:
            pass

        # Python 2.6+
        try:
            import multiprocessing
            return multiprocessing.cpu_count()
        except (ImportError, NotImplementedError):
            pass

        # https://github.com/giampaolo/psutil
        try:
            import psutil
            return psutil.cpu_count()  # psutil.NUM_CPUS on old versions
        except (ImportError, AttributeError):
            pass

        # POSIX
        try:
            res = int(os.sysconf('SC_NPROCESSORS_ONLN'))

            if res > 0:
                return res
        except (AttributeError, ValueError):
            pass

        # Windows
        try:
            res = int(os.environ['NUMBER_OF_PROCESSORS'])

            if res > 0:
                return res
        except (KeyError, ValueError):
            pass

        # jython
        try:
            from java.lang import Runtime
            runtime = Runtime.getRuntime()
            res = runtime.availableProcessors()
            if res > 0:
                return res
        except ImportError:
            pass

        # BSD
        try:
            sysctl = subprocess.Popen(['sysctl', '-n', 'hw.ncpu'], stdout=subprocess.PIPE)
            sc_stdout = sysctl.communicate()[0]
            res = int(sc_stdout)

            if res > 0:
                return res
        except (OSError, ValueError):
            pass

        # Linux
        try:
            with open('/proc/cpuinfo') as fh:
                res = fh.read().count('processor\t:')

            if res > 0:
                return res
        except IOError:
            pass

        # Solaris
        try:
            pseudo_devices = os.listdir('/devices/pseudo/')
            res = 0
            for pd in pseudo_devices:
                if re.match(r'^cpuid@[0-9]+$', pd):
                    res += 1

            if res > 0:
                return res
        except OSError:
            pass

        # Other UNIXes (heuristic)
        try:
            try:
                with open('/var/run/dmesg.boot') as fh:
                    dmesg = fh.read()
            except IOError:
                dmesg_process = subprocess.Popen(['dmesg'], stdout=subprocess.PIPE)
                dmesg = dmesg_process.communicate()[0]

            res = 0
            while '\ncpu' + str(res) + ':' in dmesg:
                res += 1

            if res > 0:
                return res
        except OSError:
            pass

        raise Exception('Can not determine the number of CPU(s) on this system')

