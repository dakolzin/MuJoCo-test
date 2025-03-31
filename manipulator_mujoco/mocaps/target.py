import numpy as np

class Target(object):
    """
    A class representing a pool cue with motion capture capabilities.
    """

    def __init__(self, mjcf_root):
        """
        Initializes a new instance of the PoolCueMoCap class.

        Args:
            mjcf_root: The root element of the MJCF model.
        """
        self._mjcf_root = mjcf_root

        # Add a mocap body to the worldbody
        self._mocap = self._mjcf_root.worldbody.add("body", name="mocap", mocap=True)
        self._mocap.add(
            "geom",
            name="x_axis",
            type="cylinder",
            fromto=[0, 0, 0, 0.1, 0, 0],
            size=[0.002],
            rgba=[1, 0, 0, 1],   # красный
            contype=0,
            conaffinity=0,
        )

        # Ось Y (зелёная) из (0,0,0) в (0,0.1,0)
        self._mocap.add(
            "geom",
            name="y_axis",
            type="cylinder",
            fromto=[0, 0, 0, 0, 0.1, 0],
            size=[0.002],
            rgba=[0, 1, 0, 1],
            contype=0,
            conaffinity=0,
        )

        # Ось Z (синяя) из (0,0,0) в (0,0,0.1)
        self._mocap.add(
            "geom",
            name="z_axis",
            type="cylinder",
            fromto=[0, 0, 0, 0, 0, 0.1],
            size=[0.002],
            rgba=[0, 0, 1, 1],
            contype=0,
            conaffinity=0,
        )

    @property
    def mjcf_root(self) -> object:
        """
        Gets the root element of the MJCF model.

        Returns:
            The root element of the MJCF model.
        """
        return self._mjcf_root

    @property
    def mocap(self) -> object:
        """
        Gets the mocap body.

        Returns:
            The mocap body.
        """
        return self._mocap

    def set_mocap_pose(self, physics, position=None, quaternion=None):
        """
        Sets the pose of the mocap body.

        Args:
            physics: The physics simulation.
            position: The position of the mocap body.
            quaternion: The quaternion orientation of the mocap body.
        """

        # flip quaternion xyzw to wxyz
        quaternion = np.roll(np.array(quaternion), 1)

        if position is not None:
            physics.bind(self.mocap).mocap_pos[:] = position
        if quaternion is not None:
            physics.bind(self.mocap).mocap_quat[:] = quaternion

    def get_mocap_pose(self, physics):
        
        position = physics.bind(self.mocap).mocap_pos[:]
        quaternion = physics.bind(self.mocap).mocap_quat[:]

        # flip quaternion wxyz to xyzw
        quaternion = np.roll(np.array(quaternion), -1)

        pose = np.concatenate([position, quaternion])

        return pose