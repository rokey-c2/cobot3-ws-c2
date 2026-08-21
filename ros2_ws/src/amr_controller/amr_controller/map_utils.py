"""ROS에 의존하지 않는 OccupancyGrid 데이터 생성 함수."""


def build_occupancy_grid(width_cells, height_cells, border_cells):
    """빈 직사각형 지도와 점유된 외곽 경계를 행 우선 배열로 만든다."""

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

    return data

