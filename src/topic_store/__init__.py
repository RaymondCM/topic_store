#  Raymond Kirk (Tunstill) Copyright (c) 2020
#  Email: ray.tunstill@gmail.com

from .api import load


def get_package_root(use_rospkg=True):
    import pathlib
    if use_rospkg:
        import rospkg
        return pathlib.Path(rospkg.RosPack().get_path("topic_store"))
    return (pathlib.Path(__file__) / "../../..").resolve()
