class IdTracker():
    """Abstracts out id tracking
    """
    def __init__(self):
        self.__id = 0

    def get_new_id(self) -> int:
        """Returns a unique agent id
        """
        assigned_id = self.__id
        self.__id += 1 # id will intentionally overflow to get back to 0
        return assigned_id
