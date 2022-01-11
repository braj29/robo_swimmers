from __future__ import absolute_import

from pyrevolve.sdfbuilder import Model, Posable, Link
from pyrevolve.SDF.math import Vector3


class Wall(Model):
    """
    Simple wall model to wall off the
    """

    def __init__(self, name, start, end, thickness, height, **kwargs):
        """
        Construct a wall of the given thickness and height from
        `start` to `end`. This simply results in a box.

        :param start: Starting point of the wall.
        :type start: Vector3
        :param end: Ending point of the wall.
        :type end: Vector3
        :return:
        """
        super(Wall, self).__init__(name, static=True, **kwargs)
        if start.z != end.z:
            raise AssertionError(
                    "Walls with different start / end z-axis are undefined.")

        center = 0.5 * (end + start)
        diff = end - start
        size = abs(diff)
        self.wall = Link("wall_link")
        self.wall.make_box(10e10, size, thickness, height)

        # Rotate the wall so it aligns with the vector from
        # x to y
        self.align(
            Vector3(0, 0, 0), Vector3(1, 0, 0), Vector3(0, 0, 1),
            center, diff, Vector3(0, 0, 1), Posable("mock")
        )

        self.add_element(self.wall)
