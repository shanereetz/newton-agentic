"""Physics checks: python -m unittest discover -s outputs/harmonic_drive_vbd."""
import unittest
import numpy as np
from simulate import Simulation, parser, validate_args


class ROMPhysicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sim = Simulation(parser().parse_args(['--device', 'cpu']))
        cls.solver = cls.sim.solver

    def test_affine_preload_and_finite_rotation_are_representable(self):
        s = self.solver
        angle = 1.3
        rotation = np.array([[np.cos(angle), -np.sin(angle), 0],
                             [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
        rotated = s.rest @ rotation.T
        coordinates = s.project(rotated)
        np.testing.assert_allclose(s.reconstruct(coordinates), rotated, atol=2.e-10)
        energy, gradient, min_j = s.elastic(coordinates)
        self.assertAlmostEqual(energy, 0., delta=1.e-12)
        self.assertLess(np.linalg.norm(gradient), 1.e-8)
        self.assertAlmostEqual(min_j, 1., delta=1.e-9)
        preload = s.rest * [1.033, .967, 1.]
        np.testing.assert_allclose(s.reconstruct(s.project(preload)), preload, atol=2.e-10)
        self.assertLess(s.rank, s.rest.size / 10)

    def test_elastic_gradient_matches_energy_derivative(self):
        s = self.solver
        rng = np.random.default_rng(19)
        x = s.project(s.rest * [1.02, .98, 1.])
        x += rng.normal(size=s.rank) * 1.e-6
        _, gradient, _ = s.elastic(x)
        for _ in range(5):
            direction = rng.normal(size=s.rank)
            direction /= np.linalg.norm(direction)
            delta = 1.e-7
            fd = (s.elastic(x + delta * direction)[0] - s.elastic(x - delta * direction)[0]) / (2 * delta)
            np.testing.assert_allclose(gradient @ direction, fd, rtol=2.e-5, atol=1.e-8)

    def test_unforced_rest_is_stationary(self):
        sim = self.sim
        a, b = sim.model.state(), sim.model.state()
        contacts = sim.pipeline.contacts()
        contacts.soft_contact_count.zero_()
        sim.solver.step(a, b, sim.control, contacts, 1 / 240)
        np.testing.assert_allclose(b.particle_q.numpy(), sim.rest, atol=1.e-9)
        np.testing.assert_allclose(b.particle_qd.numpy(), 0., atol=1.e-8)

    def test_contact_response_is_finite_and_preserves_kinematic_input(self):
        sim = Simulation(parser().parse_args(['--device', 'cpu', '--settle', '0']))
        sim.step()
        row = sim.rows[-1]
        self.assertGreater(row['min_volume_ratio'], 0.)
        self.assertGreater(row['input_rad'], 0.)
        self.assertLess(row['rom_residual'], 1.e-6)
        self.assertGreater(np.linalg.norm(sim.a.particle_qd.numpy()), 0.)
        np.testing.assert_allclose(sim.a.body_q.numpy()[0, 5:7],
                                   [np.sin(sim.angle / 2), np.cos(sim.angle / 2)], atol=1.e-7)

    def test_output_angle_is_continuous_across_pi(self):
        sim = self.sim
        sim.rows.clear()
        sim.frames.clear()
        for angle in (np.pi - .01, np.pi + .01):
            c, s = np.cos(angle), np.sin(angle)
            rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
            sim.a.particle_q.assign((sim.rest @ rotation.T).astype(np.float32))
            sim.capture()
        self.assertAlmostEqual(sim.rows[-1]['output_rad'] - sim.rows[-2]['output_rad'], .02, places=5)

    def test_bad_discretization_rejected(self):
        for options in (['--substeps', '0'], ['--rom-harmonics', '1'],
                        ['--rom-damping', '-1'], ['--duration', '-1'], ['--young', 'nan']):
            with self.subTest(options=options), self.assertRaises(ValueError):
                validate_args(parser().parse_args(options))


if __name__ == '__main__':
    unittest.main()
