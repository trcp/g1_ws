import pytest

from nakalab_ultralytics_api.nu_api import PersonDetectorState


def make_sample(position, confidence, stamp_ns):
    return {
        'person_pose': {
            'confedence': confidence,
            'pose_3d': {
                'pose': list(position),
            },
        },
        'position': list(position),
        'confidence': confidence,
        'stamp_ns': stamp_ns,
    }


def test_final_position_is_mean_of_temporal_cluster():
    state = PersonDetectorState.__new__(PersonDetectorState)
    state._PersonDetectorState__cluster_radius_m = 0.35
    state._PersonDetectorState__min_matched_samples = 2
    state._PersonDetectorState__selection_policy = 'densest'
    state._PersonDetectorState__matched_pose_samples = [
        make_sample([0.0, 1.0, 2.0], 0.8, 1),
        make_sample([0.1, 1.0, 2.0], 0.9, 2),
        make_sample([0.3, 1.0, 2.0], 0.7, 3),
    ]

    finalized = state._PersonDetectorState__finalize_person_poses()

    assert len(finalized) == 1
    assert finalized[0]['pose_3d']['pose'] == pytest.approx([
        0.4 / 3.0,
        1.0,
        2.0,
    ])
    assert finalized[0]['sample_count'] == 3
    assert finalized[0]['position_variance'] == pytest.approx(
        0.015555555555555555
    )
