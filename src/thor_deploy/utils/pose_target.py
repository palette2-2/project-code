"""Position-target helpers shared by deployment code and unit tests."""

import numpy as np


G1_NUM_DOFS = 29
G1_UPPER_DOF_START = 15
G1_UPPER_DOF_END = 29
G1_ACTION_SCALE = 0.25


def build_position_target(
    action,
    default_dof_pos,
    ref_upper_dof_pos=None,
    *,
    init=False,
):
    """Build absolute 29-DoF position targets.

    ``action`` is always a default-relative policy output.  When ``init`` is
    false, ``ref_upper_dof_pos`` is an absolute 14-DoF upper-body reference and
    the upper-body portion is converted to the residual convention used by
    training:

        default + 0.25 * action + (absolute_ref - default)
        == absolute_ref + 0.25 * action
    """

    action = np.asarray(action, dtype=np.float64)
    if action.ndim == 1:
        action = action.reshape(1, -1)
    default_dof_pos = np.asarray(default_dof_pos, dtype=np.float64).reshape(-1)
    if action.ndim != 2 or action.shape[1:] != (G1_NUM_DOFS,):
        raise ValueError(
            f"Expected action shape (N, {G1_NUM_DOFS}), got {action.shape}"
        )
    if default_dof_pos.shape != (G1_NUM_DOFS,):
        raise ValueError(
            f"Expected default pose shape ({G1_NUM_DOFS},), got {default_dof_pos.shape}"
        )

    target = action * G1_ACTION_SCALE + default_dof_pos
    if not init:
        if ref_upper_dof_pos is None:
            raise ValueError("ref_upper_dof_pos is required when init=False")
        ref_upper_dof_pos = np.asarray(ref_upper_dof_pos, dtype=np.float64)
        if ref_upper_dof_pos.ndim == 1:
            ref_upper_dof_pos = ref_upper_dof_pos.reshape(1, -1)
        expected_ref_shape = (action.shape[0], G1_UPPER_DOF_END - G1_UPPER_DOF_START)
        if ref_upper_dof_pos.shape != expected_ref_shape:
            raise ValueError(
                f"Expected reference shape {expected_ref_shape}, got {ref_upper_dof_pos.shape}"
            )
        target[:, G1_UPPER_DOF_START:G1_UPPER_DOF_END] += (
            ref_upper_dof_pos - default_dof_pos[G1_UPPER_DOF_START:G1_UPPER_DOF_END]
        )
    return target
