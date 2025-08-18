"""Tests for IKPy inverse kinematics implementation."""

import numpy as np
import pytest

from openarm.kinematics.inverse.ikpy import IkpyInverseKinematics


class TestIkpyInverseKinematics:
    """Test cases for IkpyInverseKinematics class."""

    @pytest.fixture
    def ik_solver(self) -> IkpyInverseKinematics:
        """Create an IkpyInverseKinematics instance for testing."""
        return IkpyInverseKinematics()

    def test_initialization(self) -> None:
        """Test initialization with default URDF path."""
        solver = IkpyInverseKinematics()
        assert solver.left_chain is not None
        assert solver.right_chain is not None
        assert solver.left_chain.name == "openarm_left_arm"
        assert solver.right_chain.name == "openarm_right_arm"

    def test_solve_left_arm_position_only(
        self, ik_solver: IkpyInverseKinematics
    ) -> None:
        """Test left arm IK with position target only."""
        target_position = np.array([0.3, 0.2, 0.4])
        joint_angles = ik_solver.solve_left_arm(target_position)

        expected = [
            0.0,
            -8.015921973847531e-11,
            -7.011177586592124e-11,
            2.331407318441673e-13,
            1.3837765947267506e-10,
            2.3314073184418727e-13,
            6.325432186426653e-12,
            -1.3478336662641326e-11,
            0.0,
            0.0,
            0.0,
            0.0,
        ]
        np.testing.assert_allclose(joint_angles, expected, atol=1e-10)

    def test_solve_right_arm_position_only(
        self, ik_solver: IkpyInverseKinematics
    ) -> None:
        """Test right arm IK with position target only."""
        target_position = np.array([0.3, -0.2, 0.4])
        joint_angles = ik_solver.solve_right_arm(target_position)

        expected = [
            0.0,
            7.99157613987405e-11,
            7.055322223002183e-11,
            2.3252824151479447e-13,
            1.3826110043031286e-10,
            2.3252824151482855e-13,
            -6.123336381555274e-12,
            1.3437399830921545e-11,
            0.0,
            0.0,
            0.0,
            0.0,
        ]
        np.testing.assert_allclose(joint_angles, expected, atol=1e-10)

    def test_solve_left_arm_with_orientation(
        self, ik_solver: IkpyInverseKinematics
    ) -> None:
        """Test left arm IK with position and orientation targets."""
        target_position = np.array([0.25, 0.15, 0.35])
        # 45-degree rotation around Z-axis
        theta = np.pi / 4
        target_orientation = np.array(
            [
                [np.cos(theta), -np.sin(theta), 0],
                [np.sin(theta), np.cos(theta), 0],
                [0, 0, 1],
            ]
        )

        joint_angles = ik_solver.solve_left_arm(target_position, target_orientation)

        expected = [
            0.0,
            -8.062843626624686e-11,
            -6.940622787710193e-11,
            2.3450600744229315e-13,
            1.3860231848136222e-10,
            2.345060074422923e-13,
            6.302685704254587e-12,
            -1.3557250296707857e-11,
            0.0,
            0.0,
            0.0,
            0.0,
        ]
        np.testing.assert_allclose(joint_angles, expected, atol=1e-10)

    def test_solve_right_arm_with_orientation(
        self, ik_solver: IkpyInverseKinematics
    ) -> None:
        """Test right arm IK with position and orientation targets."""
        target_position = np.array([0.25, -0.15, 0.35])
        # 30-degree rotation around X-axis
        theta = np.pi / 6
        target_orientation = np.array(
            [
                [1, 0, 0],
                [0, np.cos(theta), -np.sin(theta)],
                [0, np.sin(theta), np.cos(theta)],
            ]
        )

        joint_angles = ik_solver.solve_right_arm(target_position, target_orientation)

        expected = [
            0.0,
            8.042466504098012e-11,
            6.982351805703608e-11,
            2.3400799022909495e-13,
            1.3850476064727485e-10,
            2.340079902291299e-13,
            -6.019106225482146e-12,
            1.3522986383667286e-11,
            0.0,
            0.0,
            0.0,
            0.0,
        ]
        np.testing.assert_allclose(joint_angles, expected, atol=1e-10)

    def test_solve_with_initial_position(
        self, ik_solver: IkpyInverseKinematics
    ) -> None:
        """Test IK solving with initial joint position guess."""
        target_position = np.array([0.3, 0.1, 0.3])
        initial_guess = np.zeros(12)  # Chain has 12 links

        joint_angles = ik_solver.solve_left_arm(
            target_position, initial_position=initial_guess
        )

        expected = [
            0.0,
            -9.081386943634124e-11,
            -5.085907026133079e-11,
            2.6414881409061163e-13,
            1.4347877294092665e-10,
            2.641488140905943e-13,
            4.665259398807471e-12,
            -1.526987036721838e-11,
            0.0,
            0.0,
            0.0,
            0.0,
        ]
        np.testing.assert_allclose(joint_angles, expected, atol=1e-10)
