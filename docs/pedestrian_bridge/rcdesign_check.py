"""Limit-state verification using the rcdesign package.

Install the optional dependency first::

    pip install rcdesign

The helper reproduces the reinforcement arrangement of the prefabricated
girder and prints the ultimate bending moment reported by rcdesign.  The
library follows IS 456 terminology; therefore, we map the TS 500 parameters to
the closest available inputs.
"""
from __future__ import annotations

from rcdesign.is456.stressblock import LSMStressBlock
from rcdesign.is456.concrete import Concrete
from rcdesign.is456.rebar import (
    RebarHYSD,
    RebarLayer,
    RebarGroup,
    Stirrups,
    ShearRebarGroup,
)
from rcdesign.is456.section import FlangedBeamSection


def main() -> None:
    sb = LSMStressBlock("IS456 LSM")
    concrete = Concrete("M40", 40)
    steel = RebarHYSD("Fe 420", 420)

    cover = 45.0
    stirrup_diameter = 10.0
    bar_diameter = 20.0
    centroid = -(cover + stirrup_diameter + bar_diameter / 2)

    layer = RebarLayer(steel, [20, 20, 20, 20], centroid)
    shear_group = ShearRebarGroup([Stirrups(steel, 2, 10, 200)])
    section = FlangedBeamSection(
        bw=250,
        D=1020,
        bf=1500,
        Df=120,
        stress_block=sb,
        concrete=concrete,
        main_steel=RebarGroup([layer]),
        shear_steel=shear_group,
        cover=45,
    )
    xu = section.xu(0.0035)
    print(section.report(xu, 0.0035))


if __name__ == "__main__":
    main()
