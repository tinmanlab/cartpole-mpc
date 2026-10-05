"""Independent sparse state/input QP reference for the wheelbot browser MPC.

The browser receipt contains only the solved trajectory. This test rebuilds
the finite-horizon problem from the declared profile and solves it with OSQP.
"""
import json
from pathlib import Path

import numpy as np
from scipy import sparse

ROOT = Path(__file__).resolve().parents[1]
RECEIPT = ROOT / "test-results/wheelbot_mpc_reference.json"
receipt = json.loads(RECEIPT.read_text())


def solve_reference(case):
    profile = case["p"]
    A = np.asarray(profile["A"], dtype=float)
    B = np.asarray(profile["B"], dtype=float)
    Q = np.asarray(profile["Q"], dtype=float)
    R = np.asarray(profile["R"], dtype=float)
    Pterminal = np.asarray(profile["P"], dtype=float)
    uref = np.asarray(profile["uref"], dtype=float)
    e0 = np.asarray(case["e0"], dtype=float)
    limits = np.asarray(case["limits"], dtype=float)
    N = int(case["N"])
    nx, nu = A.shape[0], B.shape[1]
    state_start = 0
    input_start = nx * (N + 1)
    nvar = input_start + nu * N

    assert A.shape == (11, 11) and B.shape == (11, 3)
    assert Q.shape == (11, 11) and R.shape == (3, 3)
    assert Pterminal.shape == (11, 11)
    assert e0.shape == (11,) and uref.shape == limits.shape == (3,)
    assert N > 0 and np.all(limits > 0)

    # OSQP minimizes 1/2 z' H z + q' z. States and actual torques are
    # decision variables; stage effort is measured from the equilibrium input.
    hessian = sparse.lil_matrix((nvar, nvar), dtype=float)
    linear = np.zeros(nvar)

    def add_block(start, block):
        size = block.shape[0]
        hessian[start:start + size, start:start + size] += 2.0 * block

    for k in range(N):
        add_block(state_start + nx * k, Q)
        uidx = input_start + nu * k
        add_block(uidx, R)
        linear[uidx:uidx + nu] = -2.0 * R @ uref
    add_block(state_start + nx * N, Pterminal)

    eq_rows, eq_cols, eq_data = [], [], []
    rhs = []

    def put_block(row, col, block):
        rr, cc = np.nonzero(block)
        eq_rows.extend((row + rr).tolist())
        eq_cols.extend((col + cc).tolist())
        eq_data.extend(block[rr, cc].tolist())

    # Fix the initial state and impose x[k+1] = A x[k] + B(u[k]-uref).
    for i in range(nx):
        eq_rows.append(i)
        eq_cols.append(i)
        eq_data.append(1.0)
    rhs.extend(e0.tolist())
    for k in range(N):
        row = nx * (k + 1)
        put_block(row, nx * k, -A)
        put_block(row, nx * (k + 1), np.eye(nx))
        put_block(row, input_start + nu * k, -B)
        rhs.extend((-B @ uref).tolist())

    equalities = sparse.csc_matrix(
        (eq_data, (eq_rows, eq_cols)), shape=(nx * (N + 1), nvar)
    )
    identity_inputs = sparse.csc_matrix(
        (np.ones(nu * N),
         (np.arange(nu * N), input_start + np.arange(nu * N))),
        shape=(nu * N, nvar),
    )
    constraints = sparse.vstack((equalities, identity_inputs), format="csc")
    lower = np.r_[rhs, np.tile(-limits, N)]
    upper = np.r_[rhs, np.tile(limits, N)]

    import osqp
    solver = osqp.OSQP()
    solver.setup(
        P=sparse.triu(hessian.tocsc(), format="csc"),
        q=linear,
        A=constraints,
        l=lower,
        u=upper,
        eps_abs=1e-10,
        eps_rel=1e-10,
        max_iter=50000,
        polishing=True,
        verbose=False,
    )
    solution = solver.solve(raise_error=False)
    assert solution.info.status == "solved", (case["name"], solution.info.status)

    z = solution.x
    states = z[:input_start].reshape(N + 1, nx)
    torques = z[input_start:].reshape(N, nu)
    du = torques - uref
    cost = sum(float(x @ Q @ x + d @ R @ d)
               for x, d in zip(states[:-1], du))
    cost += float(states[-1] @ Pterminal @ states[-1])
    return states, torques, cost, int(solution.info.iter)


assert receipt["cases"], "MPC receipt contains no cases"
rows = []
active_by_horizon = {1: [], 20: []}

for case in receipt["cases"]:
    name, N = case["name"], int(case["N"])
    expected_E, expected_U, expected_J, iterations = solve_reference(case)
    result = case["result"]
    actual_E = np.asarray(result["E"], dtype=float)
    actual_U = np.asarray(result["U"], dtype=float)
    actual_u = np.asarray(result["u"], dtype=float)
    actual_J = float(result["J"])
    assert actual_E.shape == (N + 1, 11), (name, actual_E.shape)
    assert actual_U.shape == (N, 3), (name, actual_U.shape)
    assert actual_u.shape == (3,), (name, actual_u.shape)

    # Physical-unit limits catch errors hidden by normalized or condensed data.
    state_error = float(np.max(np.abs(expected_E - actual_E)))
    input_error = float(np.max(np.abs(expected_U - actual_U)))
    first_error = float(np.max(np.abs(expected_U[0] - actual_u)))
    receipt_first_error = float(np.max(np.abs(actual_U[0] - actual_u)))
    objective_error = abs(expected_J - actual_J) / max(1.0, abs(expected_J))
    assert state_error < 2e-6, (name, "state error", state_error)
    assert input_error < 2e-4, (name, "torque error Nm", input_error)
    assert first_error < 2e-4, (name, "first torque error Nm", first_error)
    assert receipt_first_error < 1e-12, (name, "U[0] and u differ", receipt_first_error)
    assert objective_error < 2e-6, (name, "relative objective error", objective_error)
    if name == "off_diagonal_R":
        assert abs(np.asarray(case["p"]["R"], dtype=float)[0, 1]) > 1e-8

    limits = np.asarray(case["limits"], dtype=float)
    utilization = np.max(np.abs(actual_U) / limits)
    active = utilization >= 1.0 - 1e-6
    if N in active_by_horizon:
        active_by_horizon[N].append(active)
    rows.append({"name": name, "N": N, "stateError": state_error,
                 "torqueErrorNm": input_error, "objectiveRelativeError": objective_error,
                 "activeInputBound": bool(active), "osqpIterations": iterations})

for N, classifications in active_by_horizon.items():
    assert any(not active for active in classifications), (N, "missing inactive case")
    assert any(active for active in classifications), (N, "missing active case")

print(json.dumps({"schema": "wheelbot-mpc-independent-reference/v1",
                  "rows": rows, "passed": True}, indent=2))
