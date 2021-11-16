from abc import ABCMeta, abstractmethod
from future.utils import with_metaclass


class BasePrimitive(with_metaclass(ABCMeta, object)):
    """This the base class for a primitive
    """
    def __init__(self):
        pass

    @abstractmethod
    def execute(self):
        pass
