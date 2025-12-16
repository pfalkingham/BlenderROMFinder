from parallel_processor_v2 import OptimizedROMProcessor
from types import SimpleNamespace


def test_debug_print_only_when_enabled(capsys):
    proc = OptimizedROMProcessor()
    proc.props = SimpleNamespace()
    proc.props.debug_mode = False

    # Should not print when debug_mode is False
    proc._debug_print("Should not see this")
    captured = capsys.readouterr()
    assert captured.out == ""

    # Should print when debug_mode True
    proc.props.debug_mode = True
    proc._debug_print("Visible debug")
    captured = capsys.readouterr()
    assert "Visible debug" in captured.out
