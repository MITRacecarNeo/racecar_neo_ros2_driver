"""Open a recorded bag for reading in whatever storage it declares (mcap or sqlite3)."""

from __future__ import annotations

from collections.abc import Iterable
from pathlib import Path
from typing import Any


def open_reader(
    path: str | Path, topics: Iterable[str] | None = None
) -> tuple[Any, dict[str, str]]:
    """
    Return a rosbag2 SequentialReader on `path` and its {topic: type name}.

    The storage plugin comes from the bag's metadata. With `topics`, the reader
    only returns those of them the bag has.
    """
    import rosbag2_py

    info = rosbag2_py.Info().read_metadata(str(path), '')
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id=info.storage_identifier),
        rosbag2_py.ConverterOptions('cdr', 'cdr'),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topics is not None:
        reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in topics if t in types]))
    return reader, types
