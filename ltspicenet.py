import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import re


def parse_asc_file(file_path):
    """
    Parses an LTSpice .asc file to extract components and wires.

    Returns:
        components (list): A list of components with their types and positions.
        wires (list): A list of wires with their start and end coordinates.
    """
    components = []
    wires = []

    with open(file_path, 'r') as file:
        lines = file.readlines()

    for line in lines:
        parts = line.strip().split()

        if len(parts) == 0:
            continue  # Skip empty lines

        # Extract wires
        if parts[0] == 'WIRE':
            x1, y1, x2, y2 = map(int, parts[1:])
            wires.append(((x1, y1), (x2, y2)))

        # Extract components
        elif parts[0] == 'SYM':
            component_type = parts[1]
            x, y = map(int, parts[2:])
            components.append({
                'type': component_type,
                'position': (x, y)
            })

        # Extracting component attributes (like resistance value, capacitance, etc.)
        elif parts[0] == 'SYMATTR':
            if len(components) > 0:
                attr_type = parts[1]
                attr_value = " ".join(parts[2:])
                components[-1][attr_type] = attr_value

    return components, wires


def draw_resistor(ax, x, y, label):
    ax.plot([x, x + 2], [y, y], color='black', lw=2)
    ax.text(x + 1, y + 0.5, label, fontsize=12, ha='center')


def draw_capacitor(ax, x, y, label):
    ax.plot([x, x + 0.5], [y, y], color='black', lw=2)
    ax.plot([x + 1.5, x + 2], [y, y], color='black', lw=2)
    ax.plot([x + 0.5, x + 0.5], [y - 0.5, y + 0.5], color='black', lw=2)
    ax.plot([x + 1.5, x + 1.5], [y - 0.5, y + 0.5], color='black', lw=2)
    ax.text(x + 1, y + 0.5, label, fontsize=12, ha='center')


def draw_diode(ax, x, y, label):
    ax.plot([x, x + 0.5], [y, y], color='black', lw=2)
    ax.plot([x + 1.5, x + 2], [y, y], color='black', lw=2)
    triangle = Polygon([[x + 0.5, y - 0.5], [x + 1.5, y], [x + 0.5, y + 0.5]], closed=True, color='black')
    ax.add_patch(triangle)
    ax.plot([x + 1.5, x + 1.5], [y - 0.5, y + 0.5], color='black', lw=2)
    ax.text(x + 1, y + 0.5, label, fontsize=12, ha='center')


def draw_wire(ax, x_start, y_start, x_end, y_end):
    ax.plot([x_start, x_end], [y_start, y_end], color='black', lw=2)


def draw_circuit(components, wires):
    fig, ax = plt.subplots(figsize=(10, 6))


    # Draw components
    for comp in components:
        x, y = comp['position']
        label = comp.get('Value', comp['type'])

        if 'Resistor' in comp['type']:
            draw_resistor(ax, x, y, label)
        elif 'Capacitor' in comp['type']:
            draw_capacitor(ax, x, y, label)
        elif 'Diode' in comp['type']:
            draw_diode(ax, x, y, label)

    # Draw wires
    for wire in wires:
        draw_wire(ax, *wire[0], *wire[1])

    # Customize the plot
    ax.set_xlim(-10, 300)
    ax.set_ylim(-10, 300)
    ax.set_aspect('equal')
    ax.axis('off')  # Turn off the axis
    plt.show()


# Example usage
file_path = 'Draft1.asc'  # Replace with the actual file path
components, wires = parse_asc_file(file_path)

# Draw the circuit
draw_circuit(components, wires)
