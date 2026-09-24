"""
Filename: hermes/aidwear/ai_fatigue/data_container.py
Author: Diwas Lamsal <diwaslamsal123@hotmail.com>
Date: 2026-04-22
Version: 1.0
Description: HERMES DataContainer for the fatigue estimation pipeline node.
    Publishes the per-inference RPE estimate together with timing metadata.
"""

from collections import OrderedDict

from hermes.base.data_container import DataContainer

from .utils.feature_extraction import all_feature_names


class FatigueEstimatorDataContainer(DataContainer):
    """Output stream for the fatigue estimator pipeline."""

    def __init__(
        self,
        modalities: list[str],
        buf_len: int,
        **_,
    ) -> None:
        super().__init__()

        self._device_name = "fatigue"
        self._modalities = modalities
        num_modalities = len(modalities)

        self._define_data_notes()

        self.add_channel(
            bundle_name=self._device_name,
            channel_name="rpe",
            data_type="float32",
            sample_size=[1],
            buf_len=buf_len,
            data_notes=self._data_notes[self._device_name]["rpe"],
        )
        for comp in ("rpe_base", "rpe_sysid"):
            self.add_channel(
                bundle_name=self._device_name,
                channel_name=comp,
                data_type="float32",
                sample_size=[1],
                buf_len=buf_len,
                data_notes=self._data_notes[self._device_name][comp],
            )
        self.add_channel(
            bundle_name=self._device_name,
            channel_name="features",
            data_type="float32",
            sample_size=[len(all_feature_names())],
            buf_len=buf_len,
            data_notes=self._data_notes[self._device_name]["features"],
        )
        self.add_channel(
            bundle_name=self._device_name,
            channel_name="num_reports",
            data_type="uint32",
            sample_size=[1],
            buf_len=buf_len,
            data_notes=self._data_notes[self._device_name]["num_reports"],
        )
        # Matching the intent stream
        self.add_channel(
            bundle_name=self._device_name,
            channel_name="toa_s",
            data_type="float64",
            sample_size=[1],
            buf_len=buf_len,
            data_notes=self._data_notes[self._device_name]["toa_s"],
        )
        self.add_channel(
            bundle_name=self._device_name,
            channel_name="sequence_id",
            data_type="uint32",
            sample_size=[1],
            buf_len=buf_len,
            data_notes=self._data_notes[self._device_name]["sequence_id"],
        )
        self.add_channel(
            bundle_name=self._device_name,
            channel_name="compute_time_s",
            data_type="float64",
            sample_size=[1],
            buf_len=buf_len,
            data_notes=self._data_notes[self._device_name]["compute_time_s"],
        )
        self.add_channel(
            bundle_name=self._device_name,
            channel_name="window_start_s",
            data_type="float64",
            sample_size=[num_modalities],
            buf_len=buf_len,
            data_notes=self._data_notes[self._device_name]["window_start_s"],
        )
        self.add_channel(
            bundle_name=self._device_name,
            channel_name="window_end_s",
            data_type="float64",
            sample_size=[num_modalities],
            buf_len=buf_len,
            data_notes=self._data_notes[self._device_name]["window_end_s"],
        )

    def get_fps(self) -> dict[str, float | None]:
        return {self._device_name: super()._get_fps(self._device_name, "rpe")}

    def _define_data_notes(self) -> None:
        self._data_notes = {self._device_name: {}}
        self._data_notes[self._device_name]["rpe"] = OrderedDict(
            [
                (
                    "Description",
                    "Predicted Rating of Perceived Exertion (Borg CR-10 scale)",
                ),
                ("Range", "[0, 10]"),
            ]
        )
        self._data_notes[self._device_name]["rpe_base"] = OrderedDict(
            [
                ("Description", "Frozen PF base RPE prediction (no personalization)"),
            ]
        )
        self._data_notes[self._device_name]["rpe_sysid"] = OrderedDict(
            [
                ("Description", "SysID report-conditioned personalized RPE prediction"),
            ]
        )
        self._data_notes[self._device_name]["features"] = OrderedDict(
            [
                (
                    "Description",
                    "Raw fatigue feature vector (pre-scaler), per inference; NaN where a modality was absent that window",
                ),
                (DataContainer.metadata_data_headings_key, all_feature_names()),
            ]
        )
        self._data_notes[self._device_name]["num_reports"] = OrderedDict(
            [
                ("Description", "Cumulative count of true RPE reports ingested so far"),
            ]
        )
        self._data_notes[self._device_name]["toa_s"] = OrderedDict(
            [
                ("Description", "Precise time when the inference completed"),
            ]
        )
        self._data_notes[self._device_name]["compute_time_s"] = OrderedDict(
            [
                ("Description", "Time in seconds the inference took"),
            ]
        )
        self._data_notes[self._device_name]["sequence_id"] = OrderedDict(
            [
                ("Description", "Sequence identifier of the inference result"),
            ]
        )
        self._data_notes[self._device_name]["window_start_s"] = OrderedDict(
            [
                (
                    "Description",
                    "Timestamps of the oldest samples across each modality, of the window ingested by the AI model",
                ),
                (DataContainer.metadata_data_headings_key, self._modalities),
            ]
        )
        self._data_notes[self._device_name]["window_end_s"] = OrderedDict(
            [
                (
                    "Description",
                    "Timestamps of the newest samples across each modality, of the window ingested by the AI model",
                ),
                (DataContainer.metadata_data_headings_key, self._modalities),
            ]
        )
