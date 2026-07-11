"""Linux abstract Unix endpoint conversion shared by client and server."""


def abstract_address(display_name):
    if not isinstance(display_name, str) or not display_name.startswith("@"):
        raise ValueError("control endpoint must start with '@'")
    name = display_name[1:]
    if not name or "\x00" in name:
        raise ValueError("abstract control endpoint name is invalid")
    return "\x00" + name
