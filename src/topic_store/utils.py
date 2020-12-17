#  Peter Lightbody (pet1330) Copyright (c) 2020
#  Email: pet1330@gmail.com


"""A decorator to cache instance variables"""
class cached_property(property):
    def __init__(self, method):
        self.method = method

    def __set__(self, obj, value):
        obj.__dict__[self.method.__name__] = value

    def __delete__(self, obj):
        del obj.__dict__[self.method.__name__]

    def __get__(self, obj, type=None):
        if obj is None:
            return self

        if not self.method.__name__ in obj.__dict__:
            obj.__dict__[self.method.__name__] = self.method(obj)
        return obj.__dict__[self.method.__name__]


class abstractstatic(staticmethod):
    __slots__ = ()

    def __init__(self, function):
        super(abstractstatic, self).__init__(function)
        function.__isabstractmethod__ = True

    __isabstractmethod__ = True
