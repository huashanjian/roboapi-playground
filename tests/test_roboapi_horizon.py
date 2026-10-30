from types import SimpleNamespace

import pytest

from policies.roboapi.horizon import (
    apply_horizon, load_task_catalog, validate_horizon, verify_horizon,
)


def test_full_catalog_covers_all_official_tasks():
    items = load_task_catalog()
    assert len(items) == 120
    assert {'BananaInBowlTask', 'RubiksCubeAndBananaTask', 'Stack3RubiksCubeTask'} <= {
        item['task_name'] for item in items
    }


def test_old_45_chunk_limit_cannot_silently_truncate_two_minutes():
    with pytest.raises(ValueError, match='budget'):
        validate_horizon(120, 45)
    assert validate_horizon(120, 90) == 3600


@pytest.mark.parametrize('original', [20, 50, 90, 120, 180, 300])
def test_override_applies_equally_to_short_and_long_tasks(original):
    cfg = SimpleNamespace(episode_length_s=original, sim=SimpleNamespace(dt=1/240), decimation=8)
    apply_horizon(cfg, 120, 90)
    evidence = verify_horizon(SimpleNamespace(max_episode_length=3600), cfg, 120, 90)
    assert evidence['original_official_episode_length_s'] == original
    assert evidence['max_episode_steps'] == 3600


def test_rejects_runtime_that_still_uses_old_timeout():
    cfg = SimpleNamespace(episode_length_s=50, sim=SimpleNamespace(dt=1/240), decimation=8)
    apply_horizon(cfg, 120, 90)
    with pytest.raises(ValueError, match='requested horizon'):
        verify_horizon(SimpleNamespace(max_episode_length=1500), cfg, 120, 90)


@pytest.mark.parametrize('seconds', [0, -1, float('inf'), float('nan'), 120.01])
def test_rejects_invalid_duration(seconds):
    with pytest.raises(ValueError):
        validate_horizon(seconds, 100)
