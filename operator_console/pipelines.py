"""Pure SRT pipeline helpers importable without GTK bindings."""


def srt_uri(host: str, port: int, latency_ms: int) -> str:
    """Return the operator-side SRT caller URI."""
    if not host or port < 1 or port > 65535 or latency_ms < 0:
        raise ValueError("invalid SRT endpoint")
    return f"srt://{host}:{port}?mode=caller&latency={latency_ms}"


def pipeline_description(host: str, port: int, latency_ms: int) -> str:
    """Low-latency SRT receiver rendered by a GTK-owned video widget."""
    uri = srt_uri(host, port, latency_ms)
    return (
        f'srtsrc uri="{uri}" ! tsdemux ! h264parse ! '
        "avdec_h264 max-threads=1 ! videoconvert ! video/x-raw,format=BGRA ! "
        "gtksink name=video_sink sync=false force-aspect-ratio=true"
    )
