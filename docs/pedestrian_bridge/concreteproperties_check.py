"""Verify the girder moment capacity with the concreteproperties package.

Install the optional dependency first::

    pip install concreteproperties

The script mirrors the geometry used in ``design/pedestrian_bridge/design.py``
and reports the ultimate moment capacity returned by the package.  The helper
uses the public API available in v0.7.0 of concreteproperties; minor
adjustments may be required if the library evolves.
"""
from __future__ import annotations

from math import pi

from concreteproperties.concrete_section import ConcreteSection
from concreteproperties.material import Concrete, Steel
from concreteproperties.pre import add_rectangular_section


def build_section() -> ConcreteSection:
    concrete = Concrete(
        name="C40/50",
        density=25.0,
        elastic_modulus=34_000.0 * 1e3,
        poissons_ratio=0.2,
        ultimate_strain=0.0035,
        compressive_strength=40.0,
    )
    steel = Steel(
        name="B420C",
        density=78.5,
        elastic_modulus=200_000.0,
        poissons_ratio=0.3,
        yield_strength=420.0,
    )

    flange_width = 1.5
    flange_thickness = 0.12
    web_width = 0.25
    web_depth = 0.9
    cover = 0.045
    stirrup_diameter = 0.01
    bar_diameter = 0.02

    section = ConcreteSection()
    section.add_element(
        add_rectangular_section(
            material=concrete,
            width=flange_width,
            depth=flange_thickness,
            location=(0.0, web_depth),
        )
    )
    section.add_element(
        add_rectangular_section(
            material=concrete,
            width=web_width,
            depth=web_depth,
            location=(0.0, 0.0),
        )
    )

    bar_area = 4 * pi * (bar_diameter / 2) ** 2
    centroid = cover + stirrup_diameter + bar_diameter / 2
    section.add_reinforcement_layer(
        material=steel,
        area=bar_area,
        location=(0.0, centroid),
    )
    return section


def main() -> None:
    section = build_section()
    result = section.moment_capacity(theta=0.0)
    print(f"Mu = {result.moment_capacity / 1e6:.1f} kNm")


if __name__ == "__main__":
    main()
