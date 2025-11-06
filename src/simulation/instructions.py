import re


def parse_instruction_file(path):
    """Parse a simple instruction file containing lines like:
    MOVETO [x, y]
    ROTATETO [angle_deg]

    ROTATETO angle is interpreted in degrees (absolute yaw, degrees).

    Returns a list of (cmd, args) tuples where args is a list of floats.
    """
    instructions = []
    try:
        with open(path, 'r', encoding='utf-8') as f:
            for raw in f:
                line = raw.strip()
                if not line or line.startswith('#'):
                    continue

                # MOVETO [x, y]
                m = re.match(r"MOVETO\s*\[\s*([\-0-9.eE+]+)\s*(?:,\s*|\s+)([\-0-9.eE+]+)\s*\]", line)
                if m:
                    x = float(m.group(1))
                    y = float(m.group(2))
                    instructions.append(('MOVETO', [x, y]))
                    continue

                # ROTATETO [angle_deg] -- degrees (absolute)
                m = re.match(r"ROTATETO\s*\[\s*([\-0-9.eE+]+)\s*\]", line)
                if m:
                    ang = float(m.group(1))
                    instructions.append(('ROTATETO', [ang]))
                    continue

                print(f"Unrecognized instruction line: {line}")
    except FileNotFoundError:
        print(f"Instruction file not found: {path}")
    return instructions
