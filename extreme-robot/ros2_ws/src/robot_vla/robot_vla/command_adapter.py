"""Parse text into the deliberately small VLA/FSM command contract."""

VALID_COMMANDS = {'CLEAN', 'PICK', 'MOVE', 'STOP', 'STOW'}
DEFAULT_TOOLS = {'CLEAN': 'cleaner', 'PICK': 'gripper', 'MOVE': 'none',
                 'STOP': 'none', 'STOW': 'none'}


def parse_text(text):
    """Return command, target object and tool; reject motor-level vocabulary."""
    fields = text.strip().split(maxsplit=1)
    if not fields:
        raise ValueError('empty command')
    command = fields[0].upper()
    if command not in VALID_COMMANDS:
        raise ValueError('command must be CLEAN, PICK, MOVE, STOP, or STOW')
    target = fields[1].strip() if len(fields) == 2 else ''
    return command, target, DEFAULT_TOOLS[command]
