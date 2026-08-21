"""ROS에 의존하지 않는 OccupancyGrid 데이터 생성 함수."""


def build_occupancy_grid(
    width_cells,
    height_cells,
    border_cells,
    occupied_rectangles=(),
):
    """외곽 경계와 고정 장애물을 포함한 행 우선 점유 지도를 만든다.

    ``occupied_rectangles`` entries use grid-cell bounds in the form
    ``(min_column, min_row, max_column, max_row)``. Maximum bounds are
    exclusive, matching Python's ``range`` semantics.
    """

    if width_cells <= 0 or height_cells <= 0:
        raise ValueError("map width and height must be positive")
    if border_cells < 0:
        raise ValueError("border_cells must not be negative")
    if border_cells * 2 >= min(width_cells, height_cells):
        raise ValueError("border must leave a free map interior")

    data = [0] * (width_cells * height_cells)

    for row in range(height_cells):
        for column in range(width_cells):
            on_border = (
                row < border_cells
                or row >= height_cells - border_cells
                or column < border_cells
                or column >= width_cells - border_cells
            )
            if on_border:
                data[row * width_cells + column] = 100

    for rectangle in occupied_rectangles:
        if len(rectangle) != 4:
            raise ValueError("occupied rectangle must have four bounds")

        min_column, min_row, max_column, max_row = rectangle
        if min_column >= max_column or min_row >= max_row:
            raise ValueError("occupied rectangle must have positive area")

        clipped_min_column = max(0, min_column)
        clipped_min_row = max(0, min_row)
        clipped_max_column = min(width_cells, max_column)
        clipped_max_row = min(height_cells, max_row)

        for row in range(clipped_min_row, clipped_max_row):
            for column in range(clipped_min_column, clipped_max_column):
                data[row * width_cells + column] = 100

    return data
