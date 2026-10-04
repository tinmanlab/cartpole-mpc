# Sequential tuning: test the method, not its name

This optional offline lane reuses the existing `DesignStudy` task evaluator and official MuJoCo WASM. It implements the previously recommended SMAC3/ConfigSpace candidate search and **SMAC's existing Intensifier**, and compares actual effect against cheaper methods. It does not implement a new optimizer, replace the controller's QP solver, or claim that every Bayesian optimization run must outperform a grid.

## Fixed comparison contract

`tests/fixtures/sequential_tuning.json` is frozen before the benchmark, SHA-256 `9ca92d14fceea5b847d75175e8edc4f51837eb920d1d35b5361929a5387accf0`. It pins the previous common task manifest, model and 600-step (12 s) rollout. The controller receives observer output, and the observer receives measurements/requested commands. Truth remains only in the simulated task evaluator. R is still fitted from the separate stationary readings; Qc shape, Qe shape and P0 remain fixed.

The search space contains LQR/hard-rail MPC, KF/EKF, continuous logarithmic effort factor [0.5,2] and effective Qe factor [0.1,10]. MPC alone has a categorical horizon in [20,30,40]. ConfigSpace's condition removes horizon from LQR configurations, rather than wasting evaluations on meaningless LQR horizons. The old nine-point API remains strict; a new explicit bounded `searchDomain` option is required by this adapter.

The original task score and physical limits are unchanged. This is a modest expansion of the old finite search domain, not another robot/problem. Horizon is a design variable, **not an evaluation fidelity**. No run is shortened to guess its final tracking result.

## Methods and budgets

Every budgeted run uses exactly 108 training-instance evaluations, including the same 12 baseline evaluations (four pair baselines x three training instances). Those evaluations are executed and charged for every method; no previous grid outcomes or cross-method caches are supplied free to SMAC.

| Method | Next configuration | Instance evaluation allocation |
|---|---|---|
| grid_budget | fixed logarithmic grid, randomized order after common initialization | all three training instances per configuration |
| random_full | ConfigSpace log-uniform random | all three instances |
| random_racing | standard SMAC proposal randomization set to probability 1 | standard SMAC Intensifier |
| smac_racing | SMAC random-forest expected improvement, interleaved with its declared random proposals | the same standard Intensifier |

There are five optimizer seeds. Execution order rotates across seeds so one method is not always run first on the shared host. A separate grid_exhaustive reference evaluates all 72 grid configurations, **216 training rollouts**. It is not an equal-budget competitor at 108, and not a continuous-space global optimum.

SMAC3 2.4.1, ConfigSpace 1.2.1 and scikit-learn 1.7.2 are pinned. The facade is `AlgorithmConfigurationFacade` with ten-tree sklearn random forest, EI and `LocalAndSortedRandomSearch` (512 challengers for this small space), four Sobol initial configurations, retraining every eight trials, and declared random interleaving probability 0.5. Random-racing changes that probability to 1 but retains SMAC infrastructure; it is not claimed to be the cheapest possible random implementation. Three one-hot instance identities let the surrogate distinguish training scenarios without using hidden plant truth. One worker is used; no distributed scheduler or external service is added.

The optimizer chooses actual instance IDs; each ID already has a fixed simulation seed. `deterministic=True` describes that repeatable mapping, not noiseless sensors. `max_config_calls=3` bounds a configuration to the three original training instances. No custom racing or statistical elimination test is handwritten.

## Preserve failure semantics when returning a scalar

SMAC receives the scalar

    16*hard_failure + 4*task_failure + completed_task_score/512.

The completed task score is checked against an analytic upper bound in the declared state/input/reference envelope; 512 is a conservative bound, not a clipping threshold. Unexpected implementation errors abort the experiment rather than masquerading as poor candidate scores. A failed/partial run has no full-duration score.

For a configuration evaluated on the same three instances, these weights strictly preserve the existing ordering: hard-failure count, final-task-failure count, then mean task score. A single hard failure outweighs all possible task/score improvements over three instances; likewise a task failure outweighs all score differences. **Partial racing averages are not complete-set certificates.** At the end only configurations actually evaluated on all three training instances can enter the two-candidate shortlist, with the LQR/KF baseline retained as a reference.

All shortlisted candidates receive the same three validation cases. Validation chooses among fully successful candidates using the prior 1% simplicity policy. That policy is not a statistical equivalence test. Test outcomes cannot change a recommendation, enlarge the search domain, or select another candidate.

## What is independent, and what is not

The training and validation templates are already known development data. The final panel keeps the six prior physical templates but replaces each seed with three new, predetermined seeds: twelve primary and six boundary cases. This separates seed realizations from selection, **not entire physical domains**.

All twenty budgeted campaigns and the additional exhaustive reference finish optimization and validation before any new test rollout. A separate immutable recommendation lock lists the only configurations the Node evaluator will accept for the test phase. The bridge refuses test calls before that lock and refuses additional training after it. This is a correctness boundary, not a hardware security interface.

Optimization seeds are the independent units for the pilot comparison. Reports include a paired percentile-bootstrap interval for each method's seed-level relative primary task-score difference against the same-seed budgeted grid, conditional on this fixed common test panel. Five repeats provide weak evidence for broad generalization. Serial frames are never counted as independent trials, and repeated identical baseline outcomes do not create a probability-of-safety certificate. Failure/task-success counts remain separate from mean score; missing full-duration comparisons are reported, not imputed as zero.

## Effect measurements

Read the generated `evidence/sequential_tuning.json` rather than assuming the optimizer is beneficial. It records:

- fully evaluated versus partially evaluated configurations and actual instance allocation;
- certified incumbent score at 12/36/72/108 calls (not a surrogate's optimistic prediction);
- new configurations, SMAC proposal origins and actual random-forest/Intensifier source identity;
- actual completed simulation steps, physical evaluation time, model/acquisition/tell/setup time, and total search wall time;
- validation calls/costs and the later test costs separately;
- locked selection, primary/boundary task outcomes and paired conditional uncertainty.

Finding the shared known baseline quickly is not a claimed improvement. More proposed configurations is not better sample efficiency unless the certified/held-out outcome is at least useful. If cheap rollouts make optimizer overhead dominate, retaining a simpler search is an admissible conclusion. Equal simulation-call budgets and equal wall-time budgets answer different questions; this experiment reports both costs but does not pretend it optimizes a universal time budget.

`grid_exhaustive` is explicitly additional effort. No measured result replaces the public defaults or automatically applies a tuning recipe. The shared page only **reads the recorded offline comparison**; Python SMAC is not running inside the browser.

## Reproduction and bounded recovery

Use a separate optional environment, not the existing native reference environment:

    python3 -m venv test-results/venv-smac
    test-results/venv-smac/bin/python -m pip install -r requirements-tuning.txt
    npm ci
    node tests/test_tuning_adapter.mjs
    test-results/venv-smac/bin/python tests/test_sequential_tuning.py

On a local terminal the monolithic study entry is `scripts/sequential_tuning.py`. For a runtime with per-job limits, use the same functions in bounded, same-source campaigns:

    for seed in 17011 17021 17027 17033 17041; do
      test-results/venv-smac/bin/python scripts/run_tuning_campaign.py --seed "$seed"
    done
    test-results/venv-smac/bin/python scripts/run_tuning_campaign.py --grid-reference
    test-results/venv-smac/bin/python scripts/run_tuning_campaign.py --finalize
    test-results/venv-smac/bin/python tests/test_tuning_reference.py
    python3 tests/test_tuning_browser.py

A completed campaign is reused only with matching source identity; an incomplete log is not silently treated as success. The final test cannot start while any required search is missing. Never run both entry styles into the same output directory. Transient run histories and compressed raw trajectories live in ignored `test-results`, not a new database. The committed summary and independent-verification receipt identify the sources and limits.

The initial smoke uncovered non-JSON NaN bounds in internal SMAC model metadata; only the stored model identity was changed. A monolithic pre-holdout attempt was cancelled when the remote 600-second execution cap was confirmed, and its partial development logs were preserved separately. Neither attempt consumed the final test panel or changed the frozen domain, score, algorithm settings or seeds. They are development overhead, not final optimizer replicates.

## Primary sources and attribution

- SMAC3 package/release and BSD-3-Clause license: https://pypi.org/project/smac/2.4.1/
- Official algorithm-configuration facade, RF, EI, Intensifier and ask/tell: https://automl.github.io/SMAC3/latest/api/smac/facade/algorithm_configuration_facade/
- ConfigSpace conditional hyperparameters: https://automl.github.io/ConfigSpace/latest/reference/conditions/
- SciPy paired bootstrap: https://docs.scipy.org/doc/scipy/reference/generated/scipy.stats.bootstrap.html

Only the task adapter, scalar task-order encoding, reproducible experiment and verification/UI wiring are local additions. The optimizer, initial design, random forest, acquisition and racing mechanisms are imported from their upstream owners. This finite control task cannot establish a universal SOTA tuner, global optimal controller/observer pair, or hardware safety guarantee.

### Statistical reading rule

The bootstrap intervals are descriptive pilot intervals conditional on the common finite test panel; they are not multiplicity-adjusted superiority claims. If all repeats choose the identical reference configuration, the resampled differences can produce a degenerate zero-width interval. That means identical recorded outcomes under this deterministic setup, **not statistical proof of equivalence in all conditions**. The predeclared 1% practical margin is a decision context, not a manufactured significance threshold. Any superiority/equivalence conclusion beyond the observed cases needs a sufficiently powered independently designed assessment.

### Development-test disclosure

One primary template/seed (`fresh-0-18101`) was exercised earlier in the bridge's phase-contract unit test after a local test lock. That test asserted only API phase behavior, not task quality; no result was fed to the optimizer or used to select settings. The formal comparison still locks every recommendation before its own test evaluations. Accordingly, describe the panel as selection-excluded fixed evaluation data, not as eighteen scenarios that had literally never been executed in any development check. Known-template/fixed-seed results do not establish independent physical-domain generalization.

## Recorded effect and adoption decision

The full frozen comparison completed **2,943 actual evaluations**: 20 budgeted searches x108 =2,160, plus216 exhaustive-reference evaluations,189 validation evaluations and378 post-lock test evaluations. This count excludes earlier development/contract-test attempts. Every formal evaluation completed its requested600 steps; final-task failures are still recorded separately. Independent reconstruction checks the raw trajectory terms, phase ordering, candidate ranks, trial budgets and conditional bootstrap.

Mean search results across the five optimizer seeds:

| Method | Search calls | Distinct configs examined | All3instances completed | Tuner overhead [s] | Total search wall [s] |
|---|---:|---:|---:|---:|---:|
| grid_budget |108|36.0|36.0|0.003|63.57|
| random_full |108|36.0|36.0|0.028|51.43|
| random_racing |108|86.8|8.0|28.10|76.52|
| smac_racing |108|83.2|9.2|38.12|88.27|
| grid_exhaustive (one extra reference)|216|72|72|0.002|116.57|

Racing examined more distinct configurations at the same instance-evaluation budget. Most of those configurations were tested on only one or two instances; it would be false to count them all as validated designs. SMAC's actual local-search acquisition proposals were exercised, and some certified training scores improved slightly over the finite grid. This did not change the validation-selected recipe.

**All21 recommendations were the same measured-R LQR/KF baseline, effort multiplier1 and effective-Qe multiplier1.** It met all12 primary tests and4/6 boundary tests. Its primary mean task score was **0.2508881301**, identical for every method. The extra216-call grid reference also retained the same recipe. Therefore no independent-test control improvement was observed and the original defaults are retained.

The chosen setup starts with a strong theory-informed baseline supplied equally to every method. Rediscovering it at a smaller budget than the exhaustive reference is not evidence of an SMAC-specific speedup: the cheaper budgeted grid/random methods also retained it. Slight training gains did not translate into an adopted improvement. The frozen task's weak differentiation between LQR and unconstrained MPC behavior further limits what can be concluded about more difficult control-design spaces.

Here SMAC's mean total search wall time was about88.3 s versus63.6 s for the budgeted grid and51.4 s for random full evaluation. Runtime reflects a shared host, differing configuration costs and measured Python/Node/recording overhead, not isolated processor benchmarking. Even so, the additional38.1 s of tuner work is explicit and must not be omitted when calling this method efficient. No universal timing ratio is inferred.

**Decision:** retain the verified SMAC/racing path as an optional tool for conditional, potentially larger or more expensive design searches; do not make it the default optimizer for this small benchmark based on its name. Reuse the existing grid/random/theory baseline when it achieves the stated task more cheaply. A later, harder task should remeasure improvement under a frozen budget, not assume that the present wider candidate coverage guarantees future benefit.

The conditional paired differences on this panel are all zero because the selected configurations and evaluation data are identical. The resulting [0,0] bootstrap intervals are degenerate descriptive results, not proof of statistical equivalence across all conditions. The source-linked offline table states this limitation and never changes the shared runtime or applies a new parameter set.

The public npm entry uses the bounded, source-checked runner: `npm run benchmark:tuning -- --seed 17011`, then the other declared seeds, `--grid-reference` and `--finalize`. It refuses incompatible or incomplete evidence reuse. The monolithic Python module is an alternative only for a clean output directory and must not be mixed into an existing bounded campaign.

### Resumed effect review: was the baseline mechanically forced?

No. The selection stage allowed nonbaseline configurations and used their actual validation scores. In four of five SMAC repeats a new continuous candidate improved the certified three-instance training score, but its validation score was worse than the retained LQR/KF baseline (0.2464637151). The new shortlisted SMAC candidates had validation scores from approximately 0.2469726884 to 0.2504888477; they were not adopted. In the fifth repeat no improved certified training candidate was found. This is observed training-to-validation behavior, not evidence of global optimality or general inferiority of Bayesian optimization. The predeclared 1% simplicity policy still explains selection between the nearly identical baseline controller/observer pairs.

The interruption happened after the full benchmark and its raw histories were already recorded. The resumed review reconstructed all 2,943 evaluation outcomes, allocations, scores and conditional statistics without rerunning or retuning the frozen campaign. The remaining long-lived verification command timed out; fresh bounded contract/reference/browser checks were then used as the verification authority. No unknown-result optimization was restarted and no extra test-driven search was performed. Optional tuner execution, local benchmark validation, exact-head CI, independent approval and publication remain separate claims.
