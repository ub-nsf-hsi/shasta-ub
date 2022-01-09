from abc import ABCMeta, abstractmethod
from future.utils import with_metaclass


class BaseActor(with_metaclass(ABCMeta, object)):
    """This the base class for single UGV robot
    """
    physics_client = None
    _loaded = False

    def __init__(self):
        self.states = {}
        self._actor_id = None
        self.init_pos = None
        self.init_orientation = None

    def _load(self):
        """Load object into pybullet and return list of loaded body ids."""
        if self._loaded:
            raise ValueError("Cannot load an actor multiple times.")
        self._loaded = True
        actor_ids = self.load_asset()

        if not isinstance(actor_ids, list):
            actor_ids = [actor_ids]

        self._actor_id = actor_ids[0]
        return actor_ids

    def get_actor_id(self):
        return self._actor_id

    @abstractmethod
    def load_asset(self):
        pass

    @abstractmethod
    def reset(self):
        raise NotImplementedError

    @abstractmethod
    def get_observation(self):
        raise NotImplementedError

    @abstractmethod
    def apply_action(self, action):
        raise NotImplementedError

    @abstractmethod
    def destroy(self):
        raise NotImplementedError
