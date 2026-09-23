"""Explicit communication interfaces used by closed-loop experiments."""

from .oracle_channel import OracleChannel
from .semantic_channel import SemanticChannel

__all__ = ["OracleChannel", "SemanticChannel"]
