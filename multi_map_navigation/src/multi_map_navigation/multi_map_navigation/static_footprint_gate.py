"""Conservative static-map footprint checks used before navigation starts."""

import math
import os

import numpy as np
import yaml


class StaticMapFootprintChecker:
    """Check a padded rectangular footprint against a ROS occupancy map."""

    def __init__(self, yaml_path):
        self.yaml_path = os.path.realpath(yaml_path)
        with open(self.yaml_path, encoding='utf-8') as stream:
            config = yaml.safe_load(stream)

        self.resolution = float(config['resolution'])
        origin = config.get('origin', (0.0, 0.0, 0.0))
        self.origin_x = float(origin[0])
        self.origin_y = float(origin[1])
        self.origin_yaw = float(origin[2])
        self.negate = bool(config.get('negate', 0))
        self.occupied_thresh = float(config.get('occupied_thresh', 0.65))
        self.free_thresh = float(config.get('free_thresh', 0.25))

        image_path = str(config['image'])
        if not os.path.isabs(image_path):
            image_path = os.path.join(
                os.path.dirname(self.yaml_path), image_path)
        self.image_path = os.path.realpath(image_path)
        self._pixels, self.width, self.height = self._load_pgm(
            self.image_path)

    @staticmethod
    def _read_token(stream):
        token = bytearray()
        while True:
            char = stream.read(1)
            if not char:
                raise ValueError('unexpected end of PGM header')
            if char == b'#':
                stream.readline()
                continue
            if not char.isspace():
                token.extend(char)
                break
        while True:
            char = stream.read(1)
            if not char or char.isspace():
                return bytes(token)
            token.extend(char)

    @classmethod
    def _load_pgm(cls, path):
        with open(path, 'rb') as stream:
            magic = cls._read_token(stream)
            width = int(cls._read_token(stream))
            height = int(cls._read_token(stream))
            max_value = int(cls._read_token(stream))
            data_offset = stream.tell()
        if magic != b'P5':
            raise ValueError(f'only binary P5 PGM maps are supported: {path}')
        if not 0 < max_value <= 255:
            raise ValueError(f'unsupported PGM max value {max_value}: {path}')
        expected_size = width * height
        if os.path.getsize(path) - data_offset < expected_size:
            raise ValueError(f'truncated PGM map: {path}')
        pixels = np.memmap(
            path, dtype=np.uint8, mode='r', offset=data_offset,
            shape=(height, width))
        return pixels, width, height

    def _world_to_cell(self, x, y):
        dx = x - self.origin_x
        dy = y - self.origin_y
        cos_o = math.cos(self.origin_yaw)
        sin_o = math.sin(self.origin_yaw)
        local_x = cos_o * dx + sin_o * dy
        local_y = -sin_o * dx + cos_o * dy
        return (
            math.floor(local_x / self.resolution),
            math.floor(local_y / self.resolution),
        )

    def _cell_to_world(self, cell_x, cell_y):
        local_x = (cell_x + 0.5) * self.resolution
        local_y = (cell_y + 0.5) * self.resolution
        cos_o = math.cos(self.origin_yaw)
        sin_o = math.sin(self.origin_yaw)
        return (
            self.origin_x + cos_o * local_x - sin_o * local_y,
            self.origin_y + sin_o * local_x + cos_o * local_y,
        )

    def _cell_is_free(self, cell_x, cell_y):
        if not (0 <= cell_x < self.width and 0 <= cell_y < self.height):
            return False, 'unknown'
        image_row = self.height - 1 - cell_y
        pixel = int(self._pixels[image_row, cell_x])
        occupancy = pixel / 255.0 if self.negate else (255 - pixel) / 255.0
        if occupancy > self.occupied_thresh:
            return False, 'occupied'
        if occupancy < self.free_thresh:
            return True, 'free'
        return False, 'unknown'

    def check_rectangular_footprint(
        self, x, y, yaw, *, half_length=0.43, half_width=0.28,
        padding=0.07,
    ):
        """Return ``(legal, reason)`` for every cell touched by the footprint."""
        length = float(half_length) + float(padding)
        width = float(half_width) + float(padding)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        corners = []
        for bx, by in ((length, width), (length, -width),
                       (-length, width), (-length, -width)):
            corners.append((
                x + cos_yaw * bx - sin_yaw * by,
                y + sin_yaw * bx + cos_yaw * by,
            ))
        cells = [self._world_to_cell(px, py) for px, py in corners]
        min_x = min(cell[0] for cell in cells) - 1
        max_x = max(cell[0] for cell in cells) + 1
        min_y = min(cell[1] for cell in cells) - 1
        max_y = max(cell[1] for cell in cells) + 1

        # Expanding by half a cell diagonal includes cells touched at an edge.
        cell_margin = self.resolution / math.sqrt(2.0)
        checked = 0
        for cell_y in range(min_y, max_y + 1):
            for cell_x in range(min_x, max_x + 1):
                world_x, world_y = self._cell_to_world(cell_x, cell_y)
                dx = world_x - x
                dy = world_y - y
                base_x = cos_yaw * dx + sin_yaw * dy
                base_y = -sin_yaw * dx + cos_yaw * dy
                if (abs(base_x) > length + cell_margin
                        or abs(base_y) > width + cell_margin):
                    continue
                checked += 1
                free, cell_state = self._cell_is_free(cell_x, cell_y)
                if not free:
                    return False, (
                        f'{cell_state} static cell ({cell_x}, {cell_y}) '
                        f'under padded footprint')
        if checked == 0:
            return False, 'footprint did not overlap the static map'
        return True, f'{checked} static cells are free'
