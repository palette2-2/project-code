RC_COMMAND_CHANNELS = {
    "unitree": "rc_command",
    "pico": "pico_rc_command",
}


def resolve_rc_command_source(value):
    source = str(value).strip().lower()
    if source not in RC_COMMAND_CHANNELS:
        choices = ", ".join(sorted(RC_COMMAND_CHANNELS))
        raise ValueError(
            f"Invalid RC_COMMAND_SOURCE={source!r}; expected one of: {choices}"
        )
    return source, RC_COMMAND_CHANNELS[source]
