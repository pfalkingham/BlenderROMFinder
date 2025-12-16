from parallel_processor_v2 import OptimizedROMProcessor
from mathutils import Matrix


def test_merge_worker_payload_basic():
    proc = OptimizedROMProcessor()
    proc.csv_data = [["rot_x","rot_y","rot_z","trans_x","trans_y","trans_z","Valid_pose"]]
    proc.valid_poses = []
    proc.props = type('P', (), {})()  # dummy props
    proc.props.only_export_valid_poses = False

    # Prepare fake payload
    payload = {
        'csv_rows': [[0,0,0,0,0,0,1], [1,1,1,0,0,0,0]],
        'valids': [{'pose_index': 0, 'pose_params':[0,0,0,0,0,0], 'pose_matrix': [float(v) for row in Matrix.Identity(4) for v in row], 'rx':0,'ry':0,'rz':0,'tx':0,'ty':0,'tz':0}]
    }

    proc.merge_worker_payload(payload)

    assert len(proc.csv_data) == 3
    assert len(proc.valid_poses) == 1
    assert proc.processed_poses == 2
