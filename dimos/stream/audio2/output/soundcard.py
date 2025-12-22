#!/usr/bin/env python3
# Copyright 2025 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Soundcard output node for playing audio through system speakers."""

from typing import Optional

import gi
from pydantic import Field

gi.require_version("Gst", "1.0")

from dimos.stream.audio2.base import GStreamerSinkBase
from dimos.stream.audio2.gstreamer import GStreamerNodeConfig
from dimos.stream.audio2.types import AudioSink
from dimos.stream.audio2.utils import apply_gstreamer_properties
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.stream.audio2.output.soundcard")


class SoundcardOutputConfig(GStreamerNodeConfig):
    """Configuration for soundcard output."""

    device: Optional[str] = Field(
        default=None, description="Audio device name (None = default device)"
    )
    buffer_time: Optional[int] = Field(
        default=None, description="Buffer time in microseconds (None = auto)"
    )
    latency_time: Optional[int] = Field(
        default=None, description="Latency time in microseconds (None = auto)"
    )


class SoundcardOutputNode(GStreamerSinkBase):
    """Soundcard output that plays AudioEvents through system speakers."""

    def __init__(self, config: SoundcardOutputConfig):
        super().__init__(config)
        self.config = config  # Type hint for better IDE support

    def _get_pipeline_string(self) -> str:
        """Build the soundcard output pipeline."""
        # We need to handle both raw and compressed audio
        # For compressed audio, we need to decode first
        # Add queue for buffering
        # Use audioconvert and audioresample for format flexibility
        # autoaudiosink automatically selects the best available sink
        parts = [
            "queue",
            "!",
            "decodebin",
            "!",
            "audioconvert",
            "!",
            "audioresample",
            "!",
            "autoaudiosink name=sink",
        ]

        return " ".join(parts)

    def _get_sink_name(self) -> str:
        """Get descriptive name for this sink."""
        device_str = f" [{self.config.device}]" if self.config.device else ""
        return f"SoundcardOutput{device_str}"

    def _configure_sink(self):
        """Configure the actual audio sink after pipeline creation."""
        # Try to get the actual sink from autoaudiosink
        sink = self._pipeline.get_by_name("sink")
        if sink:
            # autoaudiosink is a bin, get the actual sink inside it
            actual_sink = None

            # For autoaudiosink, we need to wait for it to be realized
            # Just use the autoaudiosink directly - it will forward properties
            sink_to_configure = sink

            # Set sync property - force False for immediate playback
            try:
                # Always set sync to False - we don't know the source timing
                sink_to_configure.set_property("sync", False)
                logger.info(f"Set sync=False (forced) on {sink_to_configure.get_name()}")
            except Exception as e:
                logger.warning(f"Failed to set sync property: {e}")

            # Set buffer-time and latency-time if supported
            if self.config.buffer_time is not None:
                try:
                    sink_to_configure.set_property("buffer-time", self.config.buffer_time)
                    logger.debug(f"Set buffer-time={self.config.buffer_time}")
                except Exception:
                    pass  # Not all sinks support this

            if self.config.latency_time is not None:
                try:
                    sink_to_configure.set_property("latency-time", self.config.latency_time)
                    logger.debug(f"Set latency-time={self.config.latency_time}")
                except Exception:
                    pass  # Not all sinks support this

            # Set any custom properties
            if self.config.properties:
                apply_gstreamer_properties(
                    sink_to_configure,
                    self.config.properties,
                    skip_properties=set(),
                    element_name="sink"
                )


def speaker(
    device: Optional[str] = None,
    buffer_time: Optional[int] = None,
    latency_time: Optional[int] = None,
    **kwargs,
) -> AudioSink:
    """Create a soundcard output sink.

    Args:
        device: Audio device name (None = default device)
        buffer_time: Buffer time in microseconds (None = auto)
        latency_time: Latency time in microseconds (None = auto)
        **kwargs: Additional arguments passed to SoundcardOutputConfig:
            - output: Output audio specification (usually not needed)
            - properties: Additional GStreamer element properties

    Returns:
        AudioSink that can subscribe to an Observable[AudioEvent]

    Examples:
        # Play audio through default speakers
        sink = speaker()
        file_input("audio.mp3")().subscribe(sink)

        # Low latency output
        sink = speaker(buffer_time=10000)  # 10ms buffer

        # Specific device
        sink = speaker(device="hw:1,0")
    """
    config = SoundcardOutputConfig(
        device=device, buffer_time=buffer_time, latency_time=latency_time, **kwargs
    )

    return SoundcardOutputNode(config)
