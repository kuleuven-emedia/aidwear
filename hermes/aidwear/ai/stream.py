"""
Filename: hermes/revalexo/ai/stream.py
Author: Maxim Yudayev <maxim.yudayev@gmail.com>
Date: 2026-03-16
Version: 1.0
Description: HERMES Stream object for the management of the corresponding
    PyTorch Pipeline Node.
"""

from collections import OrderedDict
from math import ceil, log2

from hermes.base.stream import Stream


class IntentClassifierStream(Stream):
    """A structure to store PyTorch prediction outputs."""

    def __init__(self, classes: dict[str, int], horizons: list[float], in_streams: list[str], **_) -> None:
        super().__init__()

        self._device_name = "classifier"

        self._classes = classes
        self._horizons = horizons
        self._in_streams = in_streams

        num_classes = len(classes)
        num_horizons = len(horizons)
        num_streams = len(in_streams)

        bit_width = 8 * 2 ** ceil(log2((len(classes).bit_length() + 7) // 8))

        self._define_data_notes()

        self.add_stream(
            device_name=self._device_name,
            stream_name="predictions",
            data_type=f"uint{bit_width}",
            sample_size=[num_horizons],
            data_notes=self._data_notes[self._device_name]["predictions"],
        )
        self.add_stream(
            device_name=self._device_name,
            stream_name="logits",
            data_type="float64",
            sample_size=[num_horizons, num_classes],
            data_notes=self._data_notes[self._device_name]["logits"],
        )
        self.add_stream(
            device_name=self._device_name,
            stream_name="toa_s",
            data_type="float64",
            sample_size=[1],
            data_notes=self._data_notes[self._device_name]["toa_s"],
        )
        self.add_stream(
            device_name=self._device_name,
            stream_name="sequence_id",
            data_type="int64",
            sample_size=[1],
            data_notes=self._data_notes[self._device_name]["sequence_id"],
        )
        self.add_stream(
            device_name=self._device_name,
            stream_name="compute_time_s",
            data_type="float64",
            sample_size=[1],
            data_notes=self._data_notes[self._device_name]["compute_time_s"],
        )
        self.add_stream(
            device_name=self._device_name,
            stream_name="window_start_s",
            data_type="float64",
            sample_size=[num_streams],
            data_notes=self._data_notes[self._device_name]["window_start_s"],
        )
        self.add_stream(
            device_name=self._device_name,
            stream_name="window_end_s",
            data_type="float64",
            sample_size=[num_streams],
            data_notes=self._data_notes[self._device_name]["window_end_s"],
        )

    def get_fps(self) -> dict[str, float | None]:
        return {self._device_name: super()._get_fps(self._device_name, "predictions")}

    def _define_data_notes(self) -> None:
        self._data_notes = {}
        self._data_notes.setdefault(self._device_name, {})

        self._data_notes[self._device_name]["logits"] = OrderedDict(
            [
                ("Description", "Probability vector"),
                ("Range", "[0,1]"),
                (Stream.metadata_data_headings_key, self._classes),
            ]
        )
        self._data_notes[self._device_name]["predictions"] = OrderedDict(
            [
                ("Description", "Label of the most likely class predictions"),
            ]
        )
        self._data_notes[self._device_name]["compute_time_s"] = OrderedDict(
            [
                ("Description", "Time in seconds the inference took"),
            ]
        )
        self._data_notes[self._device_name]["toa_s"] = OrderedDict(
            [
                ("Description", "Precise time when the inference completed"),
            ]
        )
        self._data_notes[self._device_name]["sequence_id"] = OrderedDict(
            [
                ("Description", "Sequence identifier of the inference result"),
            ]
        )
        self._data_notes[self._device_name]["window_start_s"] = OrderedDict(
            [
                ("Description", "Timestamps of the oldest samples across each modality, of the window ingested by the AI model"),
                (Stream.metadata_data_headings_key, self._in_streams),
            ]
        )
        self._data_notes[self._device_name]["window_end_s"] = OrderedDict(
            [
                ("Description", "Timestamps of the newest samples across each modality, of the window ingested by the AI model"),
                (Stream.metadata_data_headings_key, self._in_streams),
            ]
        )
