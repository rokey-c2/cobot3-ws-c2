class WheelSorterController:
    def route(self, destination: str):
        if destination not in ("A", "B"):
            raise ValueError("destination must be 'A' or 'B'")
        # TODO: actual sorter control
        return destination
