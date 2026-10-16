# Node Worker Simplification Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reduce generation/query lifecycle code to owned workers, necessary runtime synchronization, and cooperative stop checks, without weakening scene correctness.

**Architecture:** Keep the existing three `std::jthread` workers: one generation update and one worker for each query action. Remove repeated shutdown-result branches and duplicated exception handling; do not introduce queues, a worker framework, or a replacement agent loop. Shutdown requests stop and joins outside locks; results need not be delivered after shutdown.

**Tech Stack:** Existing C++23, ROS 2 Lyrical, `oai_api`; no new dependencies.

**Spec:** The user's requirements in this conversation, captured in Global Constraints below, supersede the earlier correctness plan's shutdown-hardening requirements.

## Global Constraints

- Keep general node logic as simple as possible. Use queues or mutexes where needed to prevent races during normal operation; no queue is needed for the existing reject-while-busy action policy.
- Stop background workers cooperatively at existing work boundaries. A blocking HTTP request or plugin call may finish before the stop signal is observed.
- Leave synchronous HTTP, existing timeout parameters/defaults/configuration, and the external `oai-api` repository unchanged.
- Preserve owned worker lifetime and joins before node data is destroyed; never restore detached threads capturing `this`.
- Preserve per-action admission serialization: one generation update, one FindObject, and one PromptLLM at a time. FindObject and PromptLLM may still overlap.
- Preserve runtime graph/cache mutexes, membership snapshots/revisions, pruning, stale-result rejection, strict answer validation, and no automatic paid retries.
- Preserve ordinary success/failure reporting and accepted client cancellation; call `canceled()` only when `is_canceling()` is true. A shutdown stop is not a client cancellation.
- No shutdown hooks, delivery guarantees, sleeps/grace periods, forced thread termination, executor refactor, exhaustive test matrix, or maintained middleware/HTTP fixture.
- Keep all work local; preserve signed history. Implementation commits must use GPG and both trailers: `GPT-6.1-Sol <noreply@openai.com>` and `LLM-FZI <llm@fzi.de>`.

## Review Focus

These are the focused local checks in the tasks below, not additional infrastructure:

1. A busy action rejects another same-type goal; after completion/failure/cancellation it accepts the next goal.
2. Worker replacement/destruction never joins while holding a mutex the worker needs; both query workers receive stop before either join.
3. A client cancellation exits at the next available checkpoint and is CANCELED, not ABORTED merely because a service/tool returned an interruption error.
4. Membership changes during inference still invalidate generation and positive/negative query results; malformed answers still abort.
5. Stop during a delayed request causes no subsequent task/tool work once a checkpoint is reached; do not require a final ROS result or a fixed shutdown deadline.

## File Scope

- Modify `semantic_scene_forests_query/src/semantic_scene_forests_query_node.cpp` and its paired `semantic_scene_forests_query_node.hpp`: admission, worker startup, execution, and cleanup only.
- Modify `semantic_scene_forests_generation/src/semantic_scene_forests_generation.cpp` and its paired `semantic_scene_forests_generation_node.hpp`: the equivalent generation lifecycle paths only.
- Update root `AGENTS.md`, generation `AGENTS.md`, and the generation/query `README.md` files with the agreed simple concurrency policy and truthful blocking-call limitation.
- Preserve query `src/graph_agent.cpp/.hpp`: `StopCheck`, `AgentStopped`, and the six tool-entry checks are the existing local escape from `oai::Agent::run()`, which has no stop-token API. Returning a tool error instead would let the model loop continue.
- No changes to tracker, manager, graph algorithms, messages, exports, parameter schemas, launch config, or external libraries. No new production files.

## Verification Setup

Use the current isolated worktree, starting at `abe88a0`; inspect status before editing and preserve unrelated work. Existing build/install prefixes below belong to this worktree and already contain its final correctness build.

```bash
source /opt/ros/lyrical/setup.bash
source /home/doberacker/repos/ros_ws/semantic_perception/colcon_ws/install/setup.bash
source /tmp/opencode/ssf-integration-baseline-install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_DOMAIN_ID=224
```

Run shell snippets in bash. Always source the intended overlay last, including when starting probe subprocesses, so old underlay libraries cannot mask changes. Run probes serially against loopback stubs, never a real model endpoint. Any extra checks or probe adaptations stay under `/tmp/opencode`; do not overwrite historical RED/GREEN evidence.

### Task 1: Simplify Query Worker Control Flow

**Files:** Query source/header above; root `AGENTS.md`; `semantic_scene_forests_query/README.md`.

**Interfaces:** Keep `executeFindObject(const std::shared_ptr<FindObjectGoal>&, std::stop_token)`, `executePromptLLM(const std::shared_ptr<PromptLLMGoal>&, std::stop_token)`, `runAgent(..., const StopCheck&)`, and the existing `GraphAgentSession` interface. Preserve the two action mutexes, active/stopping flags, and owned worker handles; they guard admission/replacement, not a delivery protocol.

- [ ] Record a baseline using `/tmp/opencode/ssf_task6_actions.py` (eight existing validation/membership cases). Extend a **local copy** with one compact action-lifecycle scenario: block a stub reply, reject a second same-type goal, release the reply, and accept a subsequent goal. Exercise both action types, including their allowed overlap, plus ordinary cancellation and failure recovery. Record baseline results rather than manufacturing a failing test for behavior that should stay unchanged.
- [ ] In `handleFindObjectAccepted()` and `handlePromptLLMAccepted()`, keep moving/joining the preceding worker outside its mutex and the final locked stop check before replacement. Replace the repeated shutdown-result construction/canceled-or-abort branches with clearing the active flag and returning. A goal caught during teardown need not receive a result.
- [ ] In each `execute*()` method, separate computing the result from completing the action. Retain preflight stop checks, catch `AgentStopped` once, and make one final decision: shutdown/context gone → return; actual client cancellation → canceled; computation/validation failure → abort; otherwise → succeed. Use an explicit completion outcome, not `result->success`, to distinguish abort from a valid FindObject not-found result, which still SUCCEEDS with `success=false`, ID `-1`.
- [ ] Keep the existing small `guardedTerminal(const std::function<void()>&)` at the single completion point in each action. This is cheap best-effort exception containment, not a delivery/retry protocol. Preserve normal computation error handling and active-flag release; no exception may escape the thread entry. Remove obsolete duplicated branches/comments and unused declarations/includes, not functional validation.
- [ ] Keep the destructor's request-stop-for-both-then-join ordering, tool-boundary checks, visuals timeout/pending-request cleanup, and runtime snapshot/validation functions unchanged. In `AGENTS.md`, add a short policy: owned workers, necessary runtime locks, cooperative checks, blocking calls accepted, no shutdown-delivery machinery.
- [ ] Build with the command below using `--packages-select semantic_scene_forests_query`; rerun eight existing cases and the local lifecycle check. Check cancellation during a delayed reply, absence of later tool calls after a stop checkpoint, and next-goal admission after failure/cancellation. No post-SIGINT result assertions.
- [ ] Format touched C++ using `~/.clang-format`; run `clang-tidy` using this worktree's compile database and project `.clang-tidy`. Review the focused diff, then make one signed local commit with both required trailers.

### Task 2: Simplify Generation Result Handling and Verify Together

**Files:** Generation source/header and `AGENTS.md`; `semantic_scene_forests_generation/README.md`.

**Interfaces:** Keep `executeGoal(const std::shared_ptr<GoalHandle>&, const std::stop_token&)`, `executePhase1/2(...)`, `abortGoal(...)`, `succeedGoal(...)`, and `clearActiveGoal()`. Change `stopGoal` to `void stopGoal(const std::shared_ptr<GoalHandle>&, const std::stop_token&)` so it can distinguish teardown from client cancellation. No plugin interface changes; membership snapshot/commit code stays untouched.

- [ ] Record the generation baseline with `bash /tmp/opencode/ssf-task5-probe/run.sh /tmp/opencode/ssf-integration-baseline-install`. Reuse local task-7 probe logic for failure → next-goal success and actual cancellation, without adopting its shutdown-result-delivery checks.
- [ ] In `handleAccepted()`, retain synchronized admission and move/join outside the mutex. On teardown, clear the active reservation and return instead of invoking `stopGoal()` to deliver a shutdown result. Keep the final locked stop check before installing a worker.
- [ ] Keep named abort/success/stop helpers because the phase code already uses them. Remove repeated terminal try/catch/log machinery from each helper; keep a cheap context check. `stopGoal()` returns without publishing when its stop token is requested or the context is gone; otherwise it calls `canceled()` only for an actual accepted client cancellation. Update all phase/entry call sites to pass the existing token. Shutdown takes precedence over result delivery even if the client also canceled.
- [ ] Contain any escaping execution/terminal exception once at the worker entry, log without retrying terminal publication, and always call `clearActiveGoal()` afterward. Keep ordinary plugin/service/LLM failures reporting failure while running; do not turn them into silent returns. Preserve existing object/task/service-wait checkpoints and checks before graph commit. Do not add extra lifecycle state or refactor phase algorithms.
- [ ] Update the generation docs to describe cooperative stopping and waiting for current blocking work. Reduce shutdown discussion to a short limitation note, not a guarantee. Update its `AGENTS.md` so it does not imply a whole-goal shutdown deadline.
- [ ] Build both packages, rerun the generation membership probe and query checks from Task 1, and run the existing registered suite. Use a local delayed-request probe to check worker stop/join and no next task after the checkpoint, without requesting results after SIGINT. No shutdown timing quota or upstream-race repair is an acceptance criterion.
- [ ] Format/lint as in Task 1; inspect the complete diff for preserved synchronization and functional fixes. Report actual added/deleted lines and remaining mechanisms—not a promise to remove all earlier ~450 lines, some of which protect runtime correctness. Make a signed local commit with both trailers after verification.

### Build and Test Commands

From the repository root, after Verification Setup:

```bash
colcon --log-base /tmp/opencode/ssf-worker-simplification-log build --build-base /tmp/opencode/ssf-integration-baseline-build --install-base /tmp/opencode/ssf-integration-baseline-install --packages-select semantic_scene_forests_generation semantic_scene_forests_query --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source /tmp/opencode/ssf-integration-baseline-install/setup.bash
python3 /tmp/opencode/ssf_task6_actions.py
bash /tmp/opencode/ssf-task5-probe/run.sh /tmp/opencode/ssf-integration-baseline-install
colcon --log-base /tmp/opencode/ssf-worker-simplification-test-log test --build-base /tmp/opencode/ssf-integration-baseline-build --install-base /tmp/opencode/ssf-integration-baseline-install --executor sequential --return-code-on-test-failure
colcon test-result --test-result-base /tmp/opencode/ssf-integration-baseline-build --verbose
git diff --check
git diff --stat abe88a0
```

Expected: build succeeds; eight query cases and thirteen generation membership checks pass; existing 49 actual registered tests pass (colcon counts four additional CTest wrappers). Local lifecycle checks pass; no new maintained harness. Report pre-existing lint warnings separately from new diagnostics.

## Execution and Review

- [ ] User reviews this plan before implementation. Retain the previously requested `fzi-llm/coder#default` subagents and independent review unless the user changes that choice; resolve the actual available model ID rather than guessing.
- [ ] Execute the two tasks serially, with a focused review after each; final review checks the combined diff against Global Constraints, not the obsolete shutdown-hardening plan.
- [ ] Handoff includes net code reduction, what synchronization/checkpoints remain and why, actual verification results, and accepted blocking-call/upstream shutdown limitations. No push, merge, remote changes, or worktree cleanup.

**Plan self-review:** Scope covers all three production background workers found in the repository. Each review-focus item is assigned a local check above. External HTTP, other nodes, timeout configuration, and functional correctness algorithms are deliberately unchanged.
