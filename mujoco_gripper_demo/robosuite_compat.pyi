from collections.abc import Sequence
from typing import Literal
import xml.etree.ElementTree as ET


class BoxObject:
    """The subset of robosuite's BoxObject API used by this demo."""

    def __init__(
        self,
        name: str,
        size: Sequence[float] | None = ...,
        *,
        rgba: Sequence[float] | None = ...,
        friction: float | Sequence[float] | None = ...,
        joints: Literal["default"] | Sequence[dict[str, object]] | None = ...,
        obj_type: Literal["collision", "visual", "all"] = ...,
    ) -> None: ...

    def get_obj(self) -> ET.Element: ...

    @property
    def root_body(self) -> str: ...
