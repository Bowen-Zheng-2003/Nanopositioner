import ezdxf
from ezdxf.path import make_path

DXF_FILE = '/Users/bowenzheng/Downloads/Pen15.dxf'

def interpolate_points(p1, p2, num_points):
    points = []
    for i in range(num_points + 1):
        t = i / num_points
        x = p1[0] + t * (p2[0] - p1[0])
        y = p1[1] + t * (p2[1] - p1[1])
        points.append((x, y))
    return points

def extract_coordinates(doc, scale=1.0, points_per_edge=20):
    msp = doc.modelspace()
    coords = []
    for entity in msp:
        try:
            if entity.dxftype() in ('LINE', 'LWPOLYLINE', 'POLYLINE'):
                # Extract vertices:
                if entity.dxftype() == 'LINE':
                    vertices = [entity.dxf.start, entity.dxf.end]
                else:  # POLYLINE or LWPOLYLINE
                    vertices = [(point[0], point[1]) for point in entity.get_points()]
                
                # For closed polyline, ensure loop closure for interpolation:
                closed = entity.closed if hasattr(entity, 'closed') else False
                if closed and vertices[0] != vertices[-1]:
                    vertices.append(vertices[0])
                
                # Interpolate points along edges
                for i in range(len(vertices) - 1):
                    p1 = vertices[i]
                    p2 = vertices[i+1]
                    interp_points = interpolate_points(p1, p2, points_per_edge)
                    for p in interp_points:
                        coords.append((p[0] * scale, p[1] * scale))
        except Exception:  # Broad to skip unsupported entities
            continue
    return coords


def main():
    print(f"Reading DXF file: {DXF_FILE}")
    doc = ezdxf.readfile(DXF_FILE)
    xy_points = extract_coordinates(doc, scale=1000)

    # Add first coordinate (origin)
    xy_points.insert(0, (0, 0))

    # Format output
    formatted_output = "(\n"
    for (x, y) in xy_points:
        formatted_output += f"  {{{int(x)}, {int(y)}, 0, 0}},\n"
    formatted_output = formatted_output.rstrip(",\n") + "\n)"

    print(formatted_output)

if __name__ == "__main__":
    main()
