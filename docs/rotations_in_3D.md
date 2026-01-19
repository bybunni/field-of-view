# 3D Rotations

Summary of 3D rotation concepts based on [Dances with Code Engineering Notes](https://www.danceswithcode.net/engineeringnotes/rotations_in_3d/rotations_in_3d_part1.html).

## Conventions Used

This document uses the **Tait-Bryan convention** with the following characteristics:

| Property | Value |
|----------|-------|
| Rotation Order | Z-Y-X (yaw, pitch, roll) |
| Rotation Type | Intrinsic (axes move with each rotation) |
| Rotation Direction | Active (point rotates relative to coordinate system) |
| Coordinate System | Right-handed Cartesian with right-handed rotations |

### Nomenclature

- **Yaw (w)**: Rotation about the z-axis
- **Pitch (v)**: Rotation about the y-axis
- **Roll (u)**: Rotation about the x-axis

This mirrors aircraft navigation conventions.

## Elemental Rotation Matrices

A rotation matrix transforms a point via: **p' = R · p**

### Roll (X-axis rotation by angle u)

$$
R_x(u) = \begin{bmatrix}
1 & 0 & 0 \\
0 & \cos(u) & -\sin(u) \\
0 & \sin(u) & \cos(u)
\end{bmatrix}
$$

```python
import numpy as np

def rotation_x(u):
    """Roll: rotation about X-axis by angle u (radians)."""
    c, s = np.cos(u), np.sin(u)
    return np.array([
        [1, 0,  0],
        [0, c, -s],
        [0, s,  c]
    ])
```

### Pitch (Y-axis rotation by angle v)

$$
R_y(v) = \begin{bmatrix}
\cos(v) & 0 & \sin(v) \\
0 & 1 & 0 \\
-\sin(v) & 0 & \cos(v)
\end{bmatrix}
$$

```python
def rotation_y(v):
    """Pitch: rotation about Y-axis by angle v (radians)."""
    c, s = np.cos(v), np.sin(v)
    return np.array([
        [ c, 0, s],
        [ 0, 1, 0],
        [-s, 0, c]
    ])
```

### Yaw (Z-axis rotation by angle w)

$$
R_z(w) = \begin{bmatrix}
\cos(w) & -\sin(w) & 0 \\
\sin(w) & \cos(w) & 0 \\
0 & 0 & 1
\end{bmatrix}
$$

```python
def rotation_z(w):
    """Yaw: rotation about Z-axis by angle w (radians)."""
    c, s = np.cos(w), np.sin(w)
    return np.array([
        [c, -s, 0],
        [s,  c, 0],
        [0,  0, 1]
    ])
```

## Combined Rotation Matrix (Euler to Matrix)

For intrinsic Z-Y-X rotation order, the combined rotation matrix is:

$$
R = R_z(w) \cdot R_y(v) \cdot R_x(u)
$$

The resulting matrix elements are:

$$
R = \begin{bmatrix}
c_v c_w & s_u s_v c_w - c_u s_w & s_u s_w + c_u s_v c_w \\
c_v s_w & c_u c_w + s_u s_v s_w & c_u s_v s_w - s_u c_w \\
-s_v & s_u c_v & c_u c_v
\end{bmatrix}
$$

Where $c$ = cosine, $s$ = sine.

```python
def euler_to_matrix(roll, pitch, yaw):
    """
    Convert Euler angles to rotation matrix.

    Parameters:
        roll (float): Rotation about X-axis (u) in radians
        pitch (float): Rotation about Y-axis (v) in radians
        yaw (float): Rotation about Z-axis (w) in radians

    Returns:
        np.ndarray: 3x3 rotation matrix
    """
    cu, su = np.cos(roll), np.sin(roll)
    cv, sv = np.cos(pitch), np.sin(pitch)
    cw, sw = np.cos(yaw), np.sin(yaw)

    return np.array([
        [cv*cw, su*sv*cw - cu*sw, su*sw + cu*sv*cw],
        [cv*sw, cu*cw + su*sv*sw, cu*sv*sw - su*cw],
        [-sv,   su*cv,            cu*cv           ]
    ])
```

## Extracting Euler Angles from Rotation Matrix

Given rotation matrix R, extract the angles using inverse trigonometric functions:

$$
\begin{aligned}
\text{yaw } (w) &= \text{atan2}(r_{21}, r_{11}) \\
\text{pitch } (v) &= -\arcsin(r_{31}) \\
\text{roll } (u) &= \text{atan2}(r_{32}, r_{33})
\end{aligned}
$$

```python
def matrix_to_euler(R):
    """
    Extract Euler angles from rotation matrix.

    Parameters:
        R (np.ndarray): 3x3 rotation matrix

    Returns:
        tuple: (roll, pitch, yaw) in radians
    """
    # Check for gimbal lock
    if np.abs(R[2, 0]) >= 1.0 - 1e-10:
        # Gimbal lock: pitch is ±90°
        yaw = 0.0  # Arbitrarily set yaw to zero
        if R[2, 0] < 0:  # pitch = +90°
            pitch = np.pi / 2
            roll = np.arctan2(R[0, 1], R[0, 2])
        else:  # pitch = -90°
            pitch = -np.pi / 2
            roll = np.arctan2(-R[0, 1], -R[0, 2])
    else:
        # Normal case
        pitch = -np.arcsin(R[2, 0])
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])

    return roll, pitch, yaw
```

## Gimbal Lock

**Gimbal lock** occurs when pitch = ±90°, causing cos(v) = 0. At this point, the yaw and roll axes become aligned, resulting in a loss of one degree of freedom.

When gimbal lock is detected:
- Any orientation can be described using an infinite number of yaw and roll angle combinations
- Recovery involves setting one angle (usually yaw) to zero and solving for the other

```python
def is_gimbal_locked(pitch, tolerance=1e-6):
    """Check if a pitch angle causes gimbal lock."""
    return np.abs(np.abs(pitch) - np.pi/2) < tolerance
```

## Properties of Rotation Matrices

Valid rotation matrices satisfy:

1. **Orthonormality**: All row and column vectors have unit magnitude
2. **Determinant**: det(R) = 1
3. **Inverse equals transpose**: R⁻¹ = Rᵀ

```python
def is_valid_rotation_matrix(R, tolerance=1e-6):
    """Verify that R is a valid rotation matrix."""
    # Check orthogonality: R @ R.T should equal identity
    should_be_identity = R @ R.T
    identity = np.eye(3)
    orthogonal = np.allclose(should_be_identity, identity, atol=tolerance)

    # Check determinant equals 1 (not -1, which would be a reflection)
    det_one = np.abs(np.linalg.det(R) - 1.0) < tolerance

    return orthogonal and det_one
```

## Applying Rotations

To rotate a point by a rotation matrix:

```python
def rotate_point(R, point):
    """
    Rotate a point using a rotation matrix.

    Parameters:
        R (np.ndarray): 3x3 rotation matrix
        point (np.ndarray): Point as (x, y, z) array

    Returns:
        np.ndarray: Rotated point
    """
    return R @ point


def rotate_points(R, points):
    """
    Rotate multiple points using a rotation matrix.

    Parameters:
        R (np.ndarray): 3x3 rotation matrix
        points (np.ndarray): Points as Nx3 array

    Returns:
        np.ndarray: Rotated points as Nx3 array
    """
    return (R @ points.T).T
```

## Inverse Rotation

Since R⁻¹ = Rᵀ for rotation matrices, inverse rotations are computationally efficient:

```python
def inverse_rotation(R):
    """Get the inverse rotation matrix (transpose)."""
    return R.T
```

## Matrix Concatenation

For sequential rotations, multiply matrices. If **A** is the current orientation and **B** is the next relative rotation:

$$
C = A \cdot B
$$

```python
def concatenate_rotations(R1, R2):
    """
    Concatenate two rotations.
    First R1 is applied, then R2.

    Parameters:
        R1 (np.ndarray): First rotation matrix
        R2 (np.ndarray): Second rotation matrix

    Returns:
        np.ndarray: Combined rotation matrix
    """
    return R2 @ R1
```

## Interpolation and Averaging

Euler angles **cannot reliably interpolate or average** due to discontinuities and gimbal lock. For smooth interpolation:

- **SLERP** (Spherical Linear Interpolation) works with rotation matrices but is computationally expensive
- **Quaternions** are the preferred representation for interpolation and averaging operations

```python
def lerp_rotation_matrices(R1, R2, t):
    """
    Linear interpolation between rotation matrices (naive approach).
    Note: This is NOT proper SLERP and may produce invalid results.
    Use quaternions for proper interpolation.

    Parameters:
        R1, R2: Rotation matrices
        t: Interpolation parameter [0, 1]

    Returns:
        np.ndarray: Interpolated (approximate) rotation matrix
    """
    # Naive linear interpolation - not recommended for production
    R_interp = (1 - t) * R1 + t * R2

    # Re-orthogonalize using SVD
    U, _, Vt = np.linalg.svd(R_interp)
    return U @ Vt
```

## Complete Example

```python
import numpy as np

# Create rotation from Euler angles
roll = np.radians(30)   # 30° about X
pitch = np.radians(45)  # 45° about Y
yaw = np.radians(60)    # 60° about Z

# Convert to rotation matrix
R = euler_to_matrix(roll, pitch, yaw)
print("Rotation matrix:\n", R)

# Verify it's valid
print("Valid rotation matrix:", is_valid_rotation_matrix(R))

# Rotate a point
point = np.array([1.0, 0.0, 0.0])
rotated = rotate_point(R, point)
print("Original point:", point)
print("Rotated point:", rotated)

# Extract Euler angles back
r, p, y = matrix_to_euler(R)
print(f"Extracted angles: roll={np.degrees(r):.1f}°, pitch={np.degrees(p):.1f}°, yaw={np.degrees(y):.1f}°")

# Inverse rotation brings point back
R_inv = inverse_rotation(R)
back = rotate_point(R_inv, rotated)
print("Point after inverse rotation:", back)
```

## References

- [Rotations in 3D - Part 1](https://www.danceswithcode.net/engineeringnotes/rotations_in_3d/rotations_in_3d_part1.html)
- [Rotations in 3D - Part 2](https://www.danceswithcode.net/engineeringnotes/rotations_in_3d/rotations_in_3d_part2.html)
