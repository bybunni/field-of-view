# Field of View

Tools and documentation for understanding 3D rotations and coordinate frames.

## Contents

### Documentation

- **[docs/rotations_in_3D.md](docs/rotations_in_3D.md)** - Summary of 3D rotation concepts including:
  - Tait-Bryan Z-Y-X (yaw-pitch-roll) convention
  - Elemental rotation matrices (Rx, Ry, Rz)
  - Combined Euler-to-matrix conversion
  - Gimbal lock explanation
  - Python reference implementations

### Web Tools

- **[web/rotation_viewer.html](web/rotation_viewer.html)** - Interactive 3D rotation visualizer
  - Sliders for yaw, pitch, and roll angles
  - Real-time 3D coordinate frame visualization using Three.js
  - Live rotation matrix display
  - Gimbal lock warning at pitch = ±90°
  - NED (North-East-Down) coordinate frame convention

## Running the Rotation Viewer

The rotation viewer is a self-contained HTML file. To run it:

```bash
cd web
python3 -m http.server 8080
```

Then open http://localhost:8080/rotation_viewer.html in your browser.

Alternatively, most modern browsers can open the file directly via `file://` protocol.

## Conventions

This project uses the **Tait-Bryan convention** with:

| Property | Value |
|----------|-------|
| Rotation Order | Z-Y-X (yaw, pitch, roll) |
| Rotation Type | Intrinsic (axes move with body) |
| Rotation Direction | Active (point rotates relative to frame) |
| Coordinate System | NED (North-East-Down), right-handed |

### Axis Definitions (NED)

- **X (North)**: Forward direction, roll axis
- **Y (East)**: Right direction, pitch axis
- **Z (Down)**: Downward direction, yaw axis
