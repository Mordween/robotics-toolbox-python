#!/usr/bin/env python

import numpy as np
from spatialmath.base import trotz, transl
from roboticstoolbox import DHRobot, RevoluteMDH


class Lite6(DHRobot):
    """
    A class representing the Lite6 robot arm.

    ``Lite6()`` is a class which models a ufactory lite 6 robot and
    describes its kinematic characteristics using modified DH
    conventions.

    .. runblock:: pycon

        >>> import roboticstoolbox as rtb
        >>> robot = rtb.models.DH.Lite6()
        >>> print(robot)

    .. note::
        - SI units of metres are used.
        - The model includes a tool offset.
    """

    def __init__(self):

        # deg = np.pi/180
        mm = 1e-3

        tool_offset = (103) * mm

        # This Lite6 model is defined using modified
        # Denavit-Hartenberg parameters
        L = [
            RevoluteMDH(     
                a=0,
                d=243.3*mm,
                alpha=0,
                qlim=np.array([-2.0*np.pi, 2*np.pi]),
                m=1.169,
                I=[
                    1.45164E-03,
                    1.24E-05,
                    -6.7E-06,
                    8.873E-04,
                    1.255E-04,
                    1.31993E-03,
                ],
                G=1,
            ),
            RevoluteMDH(     #theta offset : -90°
                a=0.0,
                d=0.0,
                alpha=- np.pi / 2,
                qlim=np.array([-2.61799, 2.61799]),
                m=1.192,
                I=[
                    1.5854E-03,
                    -6.766E-06,
                    -1.15136E-03,
                    5.6097E-03,
                    1.14E-06,
                    4.85E-03,
                ],
                G=1,
            ),
            RevoluteMDH(     #theta offset : -90°
                a=200*mm,
                d=0,
                alpha=np.pi,
                qlim=np.array([-0.061087, 5.235988]),
                m=0.930,
                I=[
                    8.861E-04,
                    -3.9287E-04,
                    7.066E-05,
                    1.5785E-03,
                    -2.445E-05,
                    1.84677E-03,
                ],
                G=1,
            ),
            RevoluteMDH(
                a=87*mm,
                d=227.6*mm,
                alpha=np.pi/2,
                qlim=np.array([-2*np.pi, 2*np.pi]),
                m=1.31,
                I=[
                    3.705E-03,
                    -2.0E-06,
                    7.17E-06,
                    3.0455E-03,
                    -9.3188E-04,
                    1.5413E-03,
                ],
                G=1,
            ),
            RevoluteMDH(
                a=0,
                d=0,
                alpha=np.pi / 2,
                qlim=np.array([-2.1642, 2.1642]),
                m=0.784,
                I=[
                    5.668E-04,
                    6E-07,
                    -5.3E-06,
                    5.077E-04,
                    -4.8E-07,
                    5.3E-04,
                ],
                G=1,
            ),
            RevoluteMDH(
                a=0.0,
                d=61.5*mm,
                alpha=-np.pi / 2,
                qlim=np.array([-2*np.pi, 2*np.pi]),
                m=0.180,
                I=[
                    7.726E-05,
                    1E-06,
                    4E-07,
                    8.5665E-05,
                    -6E-07,
                    1.4814E-04,
                ],
                G=1,
            ),
        ]

        tool = transl(0, 0, tool_offset) @ trotz(-np.pi / 4)

        super().__init__(
            L,
            name="Lite6",
            manufacturer="ufactory",
            meshdir="meshes/ufactory/Lite6",
            tool=tool,
        )


        self.qr = np.array([0, 0, 0, 2.5, 0, 0])
        self.qz = np.zeros(6)

        self.addconfiguration("qr", self.qr)
        self.addconfiguration("qz", self.qz)


if __name__ == "__main__":  # pragma nocover

    lite6 = Lite6()
    print(lite6)
