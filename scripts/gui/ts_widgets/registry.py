#  Raymond Kirk (Tunstill) Copyright (c) 2021
#  Email: ray.tunstill@gmail.com

from __future__ import absolute_import, division, print_function

from inspect import getargspec


class __VisualiserRegistry:
    def __init__(self):
        self._modules = {}

    def register_visualiser(self, cls_types):
        if not isinstance(cls_types, (list, tuple)):
            cls_types = [cls_types]

        def class_registration_decorator(module_class):
            # Ensure all registry items are of the same type
            for cls_type in cls_types:
                if len(self._modules):
                    required_type = type(self._modules[list(self._modules.keys())[0]])
                    if not isinstance(module_class, required_type):
                        raise ValueError(
                            "Visualiser module '{}' must inherit from {}}".format(module_class.__name__, required_type))
                if cls_type in self._modules:
                    print("Visualiser '{}' already in the detection registry as '{}'".format(
                        cls_type, self._modules[cls_type].name)
                    )
                else:
                    print("Registering visualiser '{}' as '{}'".format(module_class.__name__, cls_type))
                    self._modules[cls_type] = module_class

            return self._modules[cls_types[0]]

        return class_registration_decorator

    def __getitem__(self, item):
        return self._modules[item]

    def __contains__(self, item):
        return item in self._modules

    def available(self):
        return tuple(self._modules.keys())

    def get_arguments(self, item):
        cls = self._modules[item]
        args, _, _, defaults = getargspec(cls.__init__)
        if defaults is None:
            defaults = []
        if args is None:
            args = []
        if "self" in args:
            args.remove("self")

        n_defaults = len(defaults)
        required_args = list(args[0:len(args) - n_defaults])
        optional_args = dict(zip(args[len(args) - n_defaults:], defaults))

        return required_args, optional_args


VISUALISER_REGISTRY = __VisualiserRegistry()
