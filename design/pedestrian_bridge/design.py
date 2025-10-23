"""Detailed calculations for the IYTE campus pedestrian bridge girder.

The script follows TS 498 load prescriptions and TS 500 design rules for a
single prefabricated T-beam spanning 10 m.  It retains the closed-form
calculations used during preliminary design but now exposes each step so that
fellow engineers can audit the process or adapt parameters.  Optional helper
functions outline how to repeat the checks with the open-source
``concreteproperties`` and ``rcdesign`` libraries that the user referenced.
These packages are not bundled with the repository; see the messages emitted
when the script runs for installation instructions.
"""
from __future__ import annotations

from dataclasses import dataclass, asdict
from math import pi, sqrt
from typing import Dict, List, Optional


# ---------------------------------------------------------------------------
# Core data structures
# ---------------------------------------------------------------------------


@dataclass
class MaterialProperties:
    """Design strengths and modifiers according to TS 500."""

    concrete_grade: str
    f_ck: float  # MPa
    gamma_c: float
    alpha_cc: float
    steel_grade: str
    f_yk: float  # MPa
    gamma_s: float
    e_cm: float  # MPa, secant modulus for deflection checks

    @property
    def f_cd(self) -> float:
        """Design concrete strength in MPa."""

        return self.alpha_cc * self.f_ck / self.gamma_c

    @property
    def f_yd(self) -> float:
        """Design steel yield strength in MPa."""

        return self.f_yk / self.gamma_s


@dataclass
class GeometricLayout:
    """Basic T-beam geometry of one prefabricated girder."""

    span: float  # m
    tributary_width: float  # m
    flange_thickness: float  # m
    web_width: float  # m
    web_depth: float  # m (beneath flange)
    cover: float  # m to stirrup outer face
    stirrup_diameter: float  # m
    bar_diameter: float  # m

    @property
    def total_depth(self) -> float:
        return self.web_depth + self.flange_thickness

    @property
    def effective_depth(self) -> float:
        return self.total_depth - (self.cover + self.stirrup_diameter + self.bar_diameter / 2)

    @property
    def flange_width(self) -> float:
        return self.tributary_width


@dataclass
class LoadBreakdown:
    girder_self_weight: float  # kN/m
    slab_self_weight: float  # kN/m
    wearing_surface: float  # kN/m
    railing_line_load: float  # kN/m
    pedestrian_load: float  # kN/m

    @property
    def permanent(self) -> float:
        return self.girder_self_weight + self.slab_self_weight + self.wearing_surface + self.railing_line_load

    @property
    def variable(self) -> float:
        return self.pedestrian_load

    @property
    def service(self) -> float:
        return self.permanent + self.variable


@dataclass
class UltimateEffects:
    line_load: float  # kN/m
    bending_moment: float  # kNm
    shear: float  # kN


@dataclass
class FlexuralResult:
    required_as: float  # mm^2
    provided_as: float  # mm^2
    design_moment: float  # kNm
    design_capacity: float  # kNm
    neutral_axis_depth: float  # mm


@dataclass
class ShearResult:
    design_shear: float  # kN
    concrete_capacity: float  # kN
    required_stirrup_ratio: float  # (Asw/s) in mm^2/mm
    provided_stirrup_ratio: float  # mm^2/mm
    stirrup_spacing: float  # mm
    vrds: float  # kN provided by stirrups
    vrdmax: float  # kN maximum shear capacity


@dataclass
class ServiceabilityResult:
    line_load: float  # kN/m
    max_deflection: float  # mm
    limit: float  # mm


@dataclass
class LibraryCheck:
    library: str
    status: str
    message: str
    mu_kNm: Optional[float] = None

    def as_dict(self) -> Dict[str, Optional[float]]:
        return {
            "library": self.library,
            "status": self.status,
            "message": self.message,
            "mu_kNm": self.mu_kNm,
        }


@dataclass
class DesignSummary:
    materials: MaterialProperties
    geometry: GeometricLayout
    loads: LoadBreakdown
    ultimate: UltimateEffects
    flexure: FlexuralResult
    shear: ShearResult
    serviceability: ServiceabilityResult
    verifications: List[LibraryCheck]

    def to_dict(self) -> Dict[str, Dict[str, float]]:
        return {
            "materials": asdict(self.materials),
            "geometry": {
                "span_m": self.geometry.span,
                "tributary_width_m": self.geometry.tributary_width,
                "flange_thickness_m": self.geometry.flange_thickness,
                "web_width_m": self.geometry.web_width,
                "web_depth_m": self.geometry.web_depth,
                "effective_depth_m": self.geometry.effective_depth,
            },
            "loads": {
                "girder_self_weight_kN_per_m": self.loads.girder_self_weight,
                "slab_self_weight_kN_per_m": self.loads.slab_self_weight,
                "wearing_surface_kN_per_m": self.loads.wearing_surface,
                "railing_line_load_kN_per_m": self.loads.railing_line_load,
                "pedestrian_load_kN_per_m": self.loads.pedestrian_load,
                "permanent_kN_per_m": self.loads.permanent,
                "variable_kN_per_m": self.loads.variable,
            },
            "ultimate": asdict(self.ultimate),
            "flexure": asdict(self.flexure),
            "shear": asdict(self.shear),
            "serviceability": asdict(self.serviceability),
            "verifications": [check.as_dict() for check in self.verifications],
        }


# ---------------------------------------------------------------------------
# Helper computations
# ---------------------------------------------------------------------------


def factored_effects(geom: GeometricLayout, loads: LoadBreakdown, gamma_g: float = 1.4, gamma_q: float = 1.6) -> UltimateEffects:
    w_ed = gamma_g * loads.permanent + gamma_q * loads.variable
    m_ed = w_ed * geom.span**2 / 8
    v_ed = w_ed * geom.span / 2
    return UltimateEffects(line_load=w_ed, bending_moment=m_ed, shear=v_ed)


def _moment_capacity(geom: GeometricLayout, mat: MaterialProperties, as_mm2: float) -> tuple[float, float]:
    """Return (M_Rd, a) in kNm and mm using TS 500 T-beam rules."""

    bf = geom.flange_width * 1000
    bw = geom.web_width * 1000
    tf = geom.flange_thickness * 1000
    d = geom.effective_depth * 1000

    fcd = mat.f_cd / 1000  # kN/mm^2
    fyd = mat.f_yd / 1000  # kN/mm^2

    num = as_mm2 * fyd
    a_rect = num / (0.85 * fcd * bf)
    if a_rect <= tf:
        a = a_rect
        c_force = 0.85 * fcd * bf * a
        mrd = c_force * (d - a / 2) / 1000
        return mrd, a

    a = (num / (0.85 * fcd) - (bf - bw) * tf) / bw
    c_flange = 0.85 * fcd * bf * tf
    c_web = 0.85 * fcd * bw * (a - tf)
    z_flange = d - tf / 2
    z_web = d - (tf + (a - tf) / 2)
    mrd = (c_flange * z_flange + c_web * z_web) / 1000
    return mrd, a


def design_flexure(geom: GeometricLayout, mat: MaterialProperties, design_moment: float, bar_area: float) -> FlexuralResult:
    bf = geom.flange_width * 1000
    fcd = mat.f_cd / 1000
    fyd = mat.f_yd / 1000
    d = geom.effective_depth * 1000

    A = fyd**2 / (2 * 0.85 * fcd * bf)
    B = -fyd * d
    C = design_moment * 1000  # kNmm
    disc = B**2 - 4 * A * C
    if disc < 0:
        raise ValueError("Neutral axis extends into the web; adjust section dimensions.")

    required_as = (-B - sqrt(disc)) / (2 * A)
    provided_as = bar_area
    capacity, a = _moment_capacity(geom, mat, provided_as)
    return FlexuralResult(
        required_as=required_as,
        provided_as=provided_as,
        design_moment=design_moment,
        design_capacity=capacity,
        neutral_axis_depth=a / 0.85,
    )


def design_shear(
    geom: GeometricLayout,
    mat: MaterialProperties,
    loads: LoadBreakdown,
    flexure: FlexuralResult,
    stirrup_diameter: float,
    legs: int,
    spacing_mm: float,
    theta_deg: float = 45.0,
) -> ShearResult:
    from math import radians, tan

    effects = factored_effects(geom, loads)
    ved = effects.shear
    bw_mm = geom.web_width * 1000
    d_mm = geom.effective_depth * 1000
    rho = flexure.provided_as / (bw_mm * d_mm)

    k = min(1 + sqrt(200 / d_mm), 2)
    vrd_c = (0.18 / mat.gamma_c) * k * (100 * rho * mat.f_ck) ** (1 / 3) * bw_mm * d_mm / 1000

    z_mm = 0.9 * d_mm
    fyd_s = mat.f_yd  # MPa
    cot_theta = 1 / tan(radians(theta_deg))
    area_stirrup = legs * pi * (stirrup_diameter * 1000) ** 2 / 4
    asw_over_s_provided = area_stirrup / spacing_mm
    asw_over_s_required = max((ved - vrd_c) * 1000 / (z_mm * fyd_s * cot_theta), 0.0)
    vrd_s = asw_over_s_provided * z_mm * fyd_s * cot_theta / 1000

    nu1 = 0.6 * (1 - mat.f_ck / 250)
    vrd_max = bw_mm * z_mm * nu1 * mat.f_cd / 1000

    return ShearResult(
        design_shear=ved,
        concrete_capacity=vrd_c,
        required_stirrup_ratio=asw_over_s_required,
        provided_stirrup_ratio=asw_over_s_provided,
        stirrup_spacing=spacing_mm,
        vrds=vrd_s,
        vrdmax=vrd_max,
    )


def gross_inertia(geom: GeometricLayout) -> float:
    bf = geom.flange_width
    tf = geom.flange_thickness
    bw = geom.web_width
    hw = geom.web_depth
    a_flange = bf * tf
    a_web = bw * hw
    y_bar = (a_flange * (hw + tf / 2) + a_web * (hw / 2)) / (a_flange + a_web)
    i_flange = bf * tf**3 / 12 + a_flange * (hw + tf / 2 - y_bar) ** 2
    i_web = bw * hw**3 / 12 + a_web * (hw / 2 - y_bar) ** 2
    return i_flange + i_web


def check_deflection(geom: GeometricLayout, loads: LoadBreakdown, materials: MaterialProperties, inertia_m4: float) -> ServiceabilityResult:
    w_service = loads.service
    L = geom.span
    e_cm = materials.e_cm * 1e6  # convert MPa to N/m^2
    delta = 5 * (w_service * 1000) * L**4 / (384 * e_cm * inertia_m4)
    limit = L * 1000 / 500
    return ServiceabilityResult(line_load=w_service, max_deflection=delta * 1000, limit=limit)


# ---------------------------------------------------------------------------
# Optional verifications with external libraries
# ---------------------------------------------------------------------------


def attempt_concreteproperties_check(geom: GeometricLayout, mat: MaterialProperties, flexure: FlexuralResult) -> LibraryCheck:
    """Provide guidance for verifying the moment capacity with concreteproperties."""

    try:
        import concreteproperties  # type: ignore  # noqa: F401
    except Exception as exc:  # pragma: no cover - depends on environment
        return LibraryCheck(
            library="concreteproperties",
            status="not-run",
            message=(
                "Package unavailable ({}). Install via `pip install concreteproperties` "
                "and run the helper snippet in docs/pedestrian_bridge/concreteproperties_check.py"
            ).format(exc),
            mu_kNm=None,
        )

    # Library is available; outline computation while guarding against API changes.
    try:  # pragma: no cover - only executed when the package exists
        from concreteproperties.material import Concrete, Steel  # type: ignore
        from concreteproperties.pre import add_rectangular_section  # type: ignore
        from concreteproperties.concrete_section import ConcreteSection  # type: ignore

        concrete = Concrete(
            name=mat.concrete_grade,
            density=25,
            elastic_modulus=mat.e_cm * 1e3,
            poissons_ratio=0.2,
            ultimate_strain=0.0035,
            compressive_strength=mat.f_ck,
        )
        steel = Steel(
            name=mat.steel_grade,
            density=78.5,
            elastic_modulus=200000,
            poissons_ratio=0.3,
            yield_strength=mat.f_yk,
        )
        section = ConcreteSection()
        # Build flange and web as separate blocks
        section.add_element(
            add_rectangular_section(
                concrete,
                width=geom.flange_width,
                depth=geom.flange_thickness,
                location=(0, geom.web_depth),
            )
        )
        section.add_element(
            add_rectangular_section(
                concrete,
                width=geom.web_width,
                depth=geom.web_depth,
                location=(0, 0),
            )
        )
        # Reinforcement layer (single row of bars)
        cover_to_centroid = geom.cover + geom.stirrup_diameter + geom.bar_diameter / 2
        section.add_reinforcement_layer(
            steel,
            area=flexure.provided_as / 1e6,
            location=(0, cover_to_centroid),
        )
        analysis = section.moment_capacity(theta=0)
        mu_kNm = analysis.moment_capacity / 1e6
        status = "ok"
        message = "Moment capacity from concreteproperties"
    except Exception as exc:  # pragma: no cover - guard against API drift
        status = "error"
        message = f"concreteproperties call failed: {exc}"
        mu_kNm = None

    return LibraryCheck(library="concreteproperties", status=status, message=message, mu_kNm=mu_kNm)


def attempt_rcdesign_check(geom: GeometricLayout, mat: MaterialProperties, flexure: FlexuralResult, shear: ShearResult) -> LibraryCheck:
    """Outline an rcdesign verification for comparison."""

    try:
        from rcdesign.is456.stressblock import LSMStressBlock  # type: ignore
        from rcdesign.is456.concrete import Concrete as RCConcrete  # type: ignore
        from rcdesign.is456.rebar import (  # type: ignore
            RebarHYSD,
            RebarLayer,
            RebarGroup,
            Stirrups,
            ShearRebarGroup,
        )
        from rcdesign.is456.section import FlangedBeamSection  # type: ignore
    except Exception as exc:  # pragma: no cover
        return LibraryCheck(
            library="rcdesign",
            status="not-run",
            message=(
                "Package unavailable ({}). Install via `pip install rcdesign` to execute the "
                "comparison script docs/pedestrian_bridge/rcdesign_check.py"
            ).format(exc),
            mu_kNm=None,
        )

    try:  # pragma: no cover - depends on optional dependency
        sb = LSMStressBlock("IS456 LSM")
        concrete = RCConcrete(f"M{int(mat.f_ck)}", mat.f_ck)
        steel = RebarHYSD(f"Fe {int(mat.f_yk)}", mat.f_yk)
        cover_mm = geom.cover * 1000
        bar_diameter_mm = geom.bar_diameter * 1000
        layer = RebarLayer(
            steel,
            bars=[int(round(bar_diameter_mm))] * 4,
            xc=-(cover_mm + geom.stirrup_diameter * 1000 + bar_diameter_mm / 2),
        )
        shear_group = ShearRebarGroup([
            Stirrups(steel, 2, int(round(geom.stirrup_diameter * 1000)), shear.stirrup_spacing),
        ])
        section = FlangedBeamSection(
            int(round(geom.web_width * 1000)),
            int(round(geom.total_depth * 1000)),
            int(round(geom.flange_width * 1000)),
            int(round(geom.flange_thickness * 1000)),
            sb,
            concrete,
            RebarGroup([layer]),
            shear_group,
            int(round(cover_mm)),
        )
        xu = section.xu(0.0035)
        report = section.report(xu, 0.0035)
        # rcdesign reports Mu in the last line; parse it for convenience
        mu_kNm = None
        for line in report.splitlines():
            if line.strip().startswith("Mu ="):
                mu_kNm = float(line.split("=")[1].split()[0])
                break
        status = "ok"
        message = "Limit-state capacity from rcdesign"
    except Exception as exc:
        status = "error"
        message = f"rcdesign call failed: {exc}"
        mu_kNm = None

    return LibraryCheck(library="rcdesign", status=status, message=message, mu_kNm=mu_kNm)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def run_design() -> DesignSummary:
    materials = MaterialProperties(
        concrete_grade="C40/50",
        f_ck=40.0,
        gamma_c=1.5,
        alpha_cc=0.85,
        steel_grade="B420C",
        f_yk=420.0,
        gamma_s=1.15,
        e_cm=34_000.0,
    )

    geometry = GeometricLayout(
        span=10.0,
        tributary_width=1.5,
        flange_thickness=0.12,
        web_width=0.25,
        web_depth=0.9,
        cover=0.045,
        stirrup_diameter=0.01,
        bar_diameter=0.02,
    )

    loads = LoadBreakdown(
        girder_self_weight=25 * geometry.web_width * geometry.web_depth,
        slab_self_weight=25 * geometry.tributary_width * geometry.flange_thickness,
        wearing_surface=1.5,
        railing_line_load=1.0,
        pedestrian_load=5.0 * geometry.tributary_width,
    )

    ultimate = factored_effects(geometry, loads)

    bar_area = 4 * pi * (geometry.bar_diameter * 1000) ** 2 / 4
    flexure = design_flexure(geometry, materials, ultimate.bending_moment, bar_area)

    shear = design_shear(
        geometry,
        materials,
        loads,
        flexure,
        stirrup_diameter=geometry.stirrup_diameter,
        legs=2,
        spacing_mm=200,
    )

    inertia = gross_inertia(geometry)
    serviceability = check_deflection(geometry, loads, materials, inertia)

    verifications = [
        attempt_concreteproperties_check(geometry, materials, flexure),
        attempt_rcdesign_check(geometry, materials, flexure, shear),
    ]

    return DesignSummary(materials, geometry, loads, ultimate, flexure, shear, serviceability, verifications)


def _print_section(title: str, payload: Dict[str, float]) -> None:
    print(f"=== {title} ===")
    for key, value in payload.items():
        print(f"{key}: {value}")
    print()


def main() -> None:
    summary = run_design()
    data = summary.to_dict()

    _print_section("Material properties", data["materials"])
    _print_section("Geometry", data["geometry"])
    _print_section("Loads per girder (kN/m)", data["loads"])
    _print_section("Ultimate limit state", data["ultimate"])
    _print_section("Flexural design", data["flexure"])
    _print_section("Shear design", data["shear"])
    _print_section("Serviceability", data["serviceability"])

    print("=== Library cross-checks ===")
    for check in summary.verifications:
        mu_str = f", Mu = {check.mu_kNm:.1f} kNm" if check.mu_kNm is not None else ""
        print(f"{check.library}: {check.status}{mu_str}\n    {check.message}")
    print()


if __name__ == "__main__":
    main()
