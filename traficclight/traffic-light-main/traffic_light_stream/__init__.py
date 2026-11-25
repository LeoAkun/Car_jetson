"""CPU-only framing and bounded handoff primitives for P5/P6."""

__all__ = ["ProtocolError"]


class ProtocolError(ValueError):
    """Raised when a wire record or configuration violates its frozen contract."""

