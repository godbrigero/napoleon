from pydantic import BaseModel


class HybridGridStruct(BaseModel):
    size_x: int
    size_y: int
    square_size_meters: float
    static_obstacles: "list[tuple[int, int]]"

    def __init__(
        self,
        size_x: int,
        size_y: int,
        square_size_meters: float,
        static_obstacles: "list[tuple[int, int]]",
    ):
        self.size_x = size_x
        self.size_y = size_y
        self.square_size_meters = square_size_meters
        self.static_obstacles = static_obstacles

    def get_grid_size(self) -> int:
        return self.size_x * self.size_y

    def get_grid_size_meters(self) -> float:
        return self.size_x * self.square_size_meters

    def get_grid_size_pixels(self) -> int:
        return self.size_x * self.size_y
