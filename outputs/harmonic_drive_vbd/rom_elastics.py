"""Reduced-order nonlinear tetrahedral elasticity for the harmonic-drive ring.

Newton supplies geometry, state and contact candidates. This local solver performs
implicit Euler in a Fourier displacement subspace, using full tet quadrature.
The reduced minimization runs on CPU even when Newton collisions run on CUDA.
"""
import numpy as np
from scipy.linalg import cholesky, solve_triangular
from scipy.optimize import minimize


def ring_basis(rest, harmonics):
    """Orthonormal Cartesian Fourier modes with radial and axial variation.

    Both wall surfaces can move independently. In-plane affine fields (including
    finite shaft rotations and the assembly preload) are contained for h >= 1.
    This is a geometric Galerkin basis, not modes trained on a VBD trajectory.
    """
    if harmonics < 2:
        raise ValueError("ROM harmonics must be at least 2")
    theta = np.arctan2(rest[:, 1], rest[:, 0])
    radius = np.linalg.norm(rest[:, :2], axis=1)
    radial = (radius - radius.mean()) / np.ptp(radius)
    axial = (rest[:, 2] - rest[:, 2].mean()) / np.ptp(rest[:, 2])
    waves = [np.ones(len(rest))]
    for k in range(1, harmonics + 1):
        waves.extend((np.cos(k * theta), np.sin(k * theta)))
    columns = []
    for axis in range(3):
        for profile in (np.ones(len(rest)), radial if axis < 2 else axial):
            for wave in waves:
                field = np.zeros_like(rest, dtype=np.float64)
                field[:, axis] = profile * wave
                columns.append(field.ravel())
    matrix = np.stack(columns, axis=1)
    u, s, _ = np.linalg.svd(matrix, full_matrices=False)
    return u[:, s > s[0] * 1.e-10]


def cofactor(f):
    return np.stack((np.cross(f[:, :, 1], f[:, :, 2]),
                     np.cross(f[:, :, 2], f[:, :, 0]),
                     np.cross(f[:, :, 0], f[:, :, 1])), axis=2)


class SolverROM:
    """Implicit Galerkin elastics with unilateral contact and lagged friction."""
    name = "ROM elastics (Fourier Galerkin, stable Neo-Hookean)"

    def __init__(self, model, rest, tets, *, harmonics=8, iterations=400,
                 tolerance=1.e-7, damping=2.0):
        self.model = model
        self.rest = np.asarray(rest, dtype=np.float64)
        self.tets = tets
        self.basis = ring_basis(self.rest, harmonics)
        self.rank = self.basis.shape[1]
        self.fields = self.basis.reshape(-1, 3, self.rank)
        self.iterations = iterations
        self.tolerance = tolerance
        self.damping = damping
        self.last_iterations = 0
        self.last_residual = 0.0
        mass = np.repeat(model.particle_mass.numpy().astype(np.float64), 3)
        self.mass = self.basis.T @ (mass[:, None] * self.basis)
        p = self.rest[tets]
        dm = np.transpose(p[:, 1:] - p[:, :1], (0, 2, 1))
        self.volume = np.linalg.det(dm) / 6
        if np.any(self.volume <= 0):
            raise ValueError("ROM requires positive rest tetrahedra")
        inv = np.linalg.inv(dm)
        modes = self.fields[tets]
        ds = np.transpose(modes[:, 1:] - modes[:, :1], (0, 2, 1, 3))
        self.df = np.einsum('tijr,tjk->tikr', ds, inv).reshape(-1, self.rank)
        materials = model.tet_materials.numpy().astype(np.float64)
        self.mu = materials[:, 0]
        self.lam = materials[:, 1] + self.mu
        self.alpha = 1 + self.mu / self.lam
        # Rest tangent preconditions the reduced solve. Finite differences here
        # are only a one-time preconditioner, never the elastic force evaluation.
        zero = np.zeros(self.rank)
        delta = 1.e-7
        self.stiffness = np.column_stack([
            (self.elastic(zero + delta * e)[1] - self.elastic(zero - delta * e)[1]) / (2 * delta)
            for e in np.eye(self.rank)
        ])
        self.stiffness = (self.stiffness + self.stiffness.T) / 2
        self._dt = None

    def project(self, positions):
        return self.basis.T @ (np.asarray(positions) - self.rest).ravel()

    def reconstruct(self, coordinates):
        return self.rest + (self.basis @ coordinates).reshape(-1, 3)

    def elastic(self, coordinates):
        f = np.eye(3) + (self.df @ coordinates).reshape(-1, 3, 3)
        cof = cofactor(f)
        j = np.sum(f[:, :, 0] * cof[:, :, 0], axis=1)
        density = (self.mu / 2 * (np.sum(f * f, axis=(1, 2)) - 3)
                   + self.lam / 2 * ((j - self.alpha)**2 - (1 - self.alpha)**2))
        stress = self.mu[:, None, None] * f + (self.lam * (j - self.alpha))[:, None, None] * cof
        gradient = self.df.T @ (self.volume[:, None, None] * stress).ravel()
        return float(self.volume @ density), gradient, float(j.min())

    def contact_data(self, state, contacts):
        count = int(contacts.soft_contact_count.numpy()[0])
        if count > contacts.soft_contact_max:
            raise RuntimeError("ROM contact buffer overflow")
        ids = contacts.soft_contact_particle.numpy()[:count]
        shapes = contacts.soft_contact_shape.numpy()[:count]
        points = contacts.soft_contact_body_pos.numpy()[:count].astype(np.float64)
        normals = contacts.soft_contact_normal.numpy()[:count].astype(np.float64)
        bodies = self.model.shape_body.numpy()[shapes]
        poses = state.body_q.numpy()
        velocities = state.body_qd.numpy()
        body_velocity = np.zeros_like(points)
        for body in np.unique(bodies):
            if body < 0:
                continue
            mask = bodies == body
            quat = poses[body, 3:7]
            local = points[mask]
            rotated = local + 2 * np.cross(quat[:3], np.cross(quat[:3], local) + quat[3] * local)
            points[mask] = rotated + poses[body, :3]
            body_velocity[mask] = velocities[body, :3] + np.cross(velocities[body, 3:], rotated)
        radius = self.model.particle_radius.numpy()[ids]
        return ids, points, normals, radius, body_velocity

    def step(self, state_in, state_out, control, contacts, dt):
        if dt <= 0:
            raise ValueError("dt must be positive")
        if self._dt != dt:
            h = self.stiffness + (1 / dt**2 + self.damping / dt) * self.mass
            self.base_hessian = h
            self._dt = dt
        old = self.project(state_in.particle_q.numpy())
        velocity = self.basis.T @ state_in.particle_qd.numpy().ravel()
        predicted = old + dt * velocity
        external = self.basis.T @ state_in.particle_f.numpy().ravel()
        ids, points, normals, radius, body_velocity = self.contact_data(state_in, contacts)
        fields = self.fields[ids]
        normal_basis = np.einsum('cir,ci->cr', fields, normals)
        rest_gap = np.sum((self.rest[ids] - points) * normals, axis=1) - radius
        ke = self.model.soft_contact_ke
        friction = self.model.soft_contact_mu * ke * np.maximum(-(rest_gap + normal_basis @ old), 0)
        tangent_fields = fields - normals[:, :, None] * normal_basis[:, None, :]
        tangent_velocity = body_velocity - normals * np.sum(body_velocity * normals, axis=1)[:, None]
        slip_offset = -dt * tangent_velocity
        epsilon = 1.e-4 * dt
        h = self.base_hessian + ke * normal_basis.T @ normal_basis
        tangent_h = tangent_fields.reshape(-1, self.rank)
        weights = np.repeat(friction / (epsilon + 1.e-5), 3)
        h += tangent_h.T @ (weights[:, None] * tangent_h)
        transform = solve_triangular(cholesky(h, lower=True).T, np.eye(self.rank), lower=False)

        def objective(y):
            x = old + transform @ y
            energy, gradient, min_j = self.elastic(x)
            if min_j <= 0:
                return np.inf, np.zeros(self.rank)
            delta = x - predicted
            change = x - old
            energy += .5 * delta @ self.mass @ delta / dt**2
            energy += .5 * self.damping / dt * change @ self.mass @ change - external @ change
            gradient += self.mass @ delta / dt**2 + self.damping / dt * self.mass @ change - external
            gap = rest_gap + normal_basis @ x
            penetration = np.minimum(gap, 0)
            energy += .5 * ke * (penetration @ penetration)
            gradient += ke * normal_basis.T @ penetration
            slip = np.einsum('cir,r->ci', tangent_fields, change) + slip_offset
            length = np.sqrt(np.sum(slip * slip, axis=1) + epsilon**2)
            energy += friction @ (length - epsilon)
            gradient += np.einsum('cir,ci->r', tangent_fields, (friction / length)[:, None] * slip)
            return energy, transform.T @ gradient

        result = minimize(objective, np.zeros(self.rank), jac=True, method='BFGS',
                          options={'maxiter': self.iterations, 'gtol': self.tolerance,
                                   'xrtol': 1.e-12})
        residual = float(np.max(np.abs(result.jac)))
        self.last_iterations = int(result.nit)
        self.last_residual = residual
        if not np.isfinite(result.fun) or residual > max(10 * self.tolerance, 1.e-6):
            raise RuntimeError(f"ROM solve did not converge: {result.message}; residual={residual:.3g}. "
                               "Increase --iterations or --substeps.")
        coordinates = old + transform @ result.x
        positions = self.reconstruct(coordinates)
        if self.elastic(coordinates)[2] <= 0 or not np.isfinite(positions).all():
            raise RuntimeError("Invalid ROM state")
        state_out.particle_q.assign(positions.astype(np.float32))
        state_out.particle_qd.assign(((self.basis @ ((coordinates - old) / dt)).reshape(-1, 3)).astype(np.float32))
        state_out.body_q.assign(state_in.body_q)
        state_out.body_qd.assign(state_in.body_qd)
