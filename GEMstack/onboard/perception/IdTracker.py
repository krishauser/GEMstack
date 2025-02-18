class IdTracker():
    """Abstracts out id tracking
    """
    def __init__(self):
        self.__ped_id = 0

    def get_new_ped_id(self) -> int:
        """Returns a unique pedestrian id
        """
        assigned_id = self.__id
        self.__ped_id += 1 # id will intentionally overflow to get back to 0
        return assigned_id
