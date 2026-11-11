"""Verification of the ISA model against tabulated standard-atmosphere values."""

import pytest

from flightsim.atmosphere import isa


def test_sea_level_values():
    atm = isa(0.0)
    assert atm.temperature == pytest.approx(288.15)
    assert atm.pressure == pytest.approx(101_325.0)
    assert atm.density == pytest.approx(1.225, rel=1e-4)
    assert atm.speed_of_sound == pytest.approx(340.29, rel=1e-4)


def test_tropopause_values():
    atm = isa(11_000.0)
    assert atm.temperature == pytest.approx(216.65)
    assert atm.pressure == pytest.approx(22_632, rel=1e-3)
    assert atm.density == pytest.approx(0.3639, rel=1e-3)


def test_stratosphere_value():
    atm = isa(20_000.0)
    assert atm.temperature == pytest.approx(216.65)
    assert atm.pressure == pytest.approx(5_474.9, rel=1e-3)


def test_density_decreases_with_altitude():
    densities = [isa(h).density for h in range(0, 20_001, 1000)]
    assert all(a > b for a, b in zip(densities, densities[1:]))


@pytest.mark.parametrize("h", [-1000.0, 25_000.0])
def test_out_of_range_altitude_raises(h):
    with pytest.raises(ValueError):
        isa(h)
