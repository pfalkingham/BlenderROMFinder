import math
from mathutils import Matrix
from parallel_processor_v2 import OptimizedROMProcessor


def test_matrix_serialization_roundtrip():
    proc = OptimizedROMProcessor()
    # Identity
    m = Matrix.Identity(4)
    lst = proc._matrix_to_list(m)
    assert len(lst) == 16
    m2 = proc._list_to_matrix(lst)
    assert m == m2

    # Random transform
    m = Matrix.Rotation(math.radians(30), 4, 'X') @ Matrix.Translation((1.0, 2.0, 3.0))
    lst = proc._matrix_to_list(m)
    m2 = proc._list_to_matrix(lst)
    assert all(abs(a - b) < 1e-9 for a, b in zip([v for row in m for v in row], [v for row in m2 for v in row]))
