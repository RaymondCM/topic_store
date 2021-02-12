#!/usr/bin/env python

#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

# LoadBalancing utilities from raytils (https://github.com/RaymondKirk/raytils, https://pypi.org/project/raytils/)
import os
import re
import subprocess

try:
    import queue
except ImportError:
    import Queue as queue

import threading

from collections import deque
from timeit import default_timer as timer


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
                self.data_retrieval_rate.tic()
                work = self.q.get()
                self.data_retrieval_rate.toc()
            except queue.Empty:
                return

            # Create dictionary of meta values to pass to the worker
            job_meta = {
                "worker_id": self.worker_id,  # This is my id
                "worker_data_retrieval_rate": self.data_retrieval_rate.get_fps(),  # I get data every n times per second
                "worker_job_processing_rate": self.processing_rate.get_fps(),  # I process data every n times per second
            }

            self.processing_rate.tic()
            work[0](*work[1], job_meta=job_meta, **work[2])
            self.processing_rate.toc()
            self.q.task_done()


class LoadBalancer(queue.Queue, object):
    """Add jobs in the form of (func, args, kwargs) to a task queue"""
    def __init__(self, maxsize=0, threads=1):
        super(LoadBalancer, self).__init__(maxsize=maxsize)
        self.max_threads = self._max_thread_count()
        self.number_of_threads = 0
        self.tasks_per_second = FPSCounter(queue_length=maxsize)
        # Number of seconds to allow queue to catch up before adding more cores
        self._thread_grace_period = 2.0
        self._last_thread_added = timer()
        self._add_thread_threshold_percent = 0.8
        for _ in range(threads):
            self._add_thread(check_grace_period=False)

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.join()

    def _add_thread(self, check_grace_period=True):
        """Function that's used internally. If the queue of jobs ever becomes full a new thread is spawned to try to
        handle the new information in realtime"""
        # Do not add more threads if the CPU cannot support it
        if self.number_of_threads >= self.max_threads:
            return

        # Grace period so cores don't quickly accelerate to system maximum
        if check_grace_period and (timer() - self._last_thread_added) <= self._thread_grace_period:
            return

        Worker(self, self.number_of_threads).start()
        self.number_of_threads += 1
        self._last_thread_added = timer()

    def add_task(self, func, args, kwargs, wait=False):
        """

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
        self.tasks_per_second.toc()
        self.tasks_per_second.tic()

        # Add a thread if queue_size growing faster than current processing or is nearing capacity
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
            # If the queue becomes full add a thread to balance the workload
            self._add_thread()  # attempt to add a thread to balance the workload
            return False

        return True

    @staticmethod
    def _max_thread_count():
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
