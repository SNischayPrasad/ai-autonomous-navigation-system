"""End-to-end test: the full autonomy stack must reach the goal without collisions."""
import pytest

from src.config import Config
from src.simulation.scenarios import build_scenario
from src.simulation.simulator import Simulation


@pytest.mark.parametrize("scenario", ["warehouse", "crosswalk"])
def test_reaches_goal_without_collision(scenario):
    result = Simulation(build_scenario(scenario), Config(), output_dir=None, render=False, video=False).run()
    assert result.final_state == "GOAL_REACHED"
    assert result.collisions == 0
    assert result.success
