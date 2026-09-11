# com-pollen-robotics

Clean-room, API-compatible actor for the **Pollen Robotics Reachy 2 developer
SDK**, in three surfaces: a `.kotoba` schema, two `.cljk` decision cores, and
a `.cljc` host.

Derived from the resource shapes published in
[`pollen-robotics/reachy2-sdk-api`](https://github.com/pollen-robotics/reachy2-sdk-api)
(`protos/`, Apache-2.0) and [docs.pollen-robotics.com](https://docs.pollen-robotics.com/).
No upstream code, generated stub or credential is reproduced here — resource
shapes and documented behaviour only.

**This is policy and bookkeeping, not control.** It never drives a motor. It
produces the record a governor ([`kotoba-lang/robotics`](https://github.com/kotoba-lang/robotics))
needs to refuse unsafe actuation *before* actuation.


## The three surfaces

| Surface | File | What lives there |
|---|---|---|
| `.kotoba` | `schema/pollen_robotics.kotoba` | The resource shapes, EAVT-mapped. 12 entities. |
| `.cljk` | `cores/reachy_lidar_safety_core.cljk` | The mobile base's lidar safety gate. |
| `.cljk` | `cores/reachy_goto_core.cljk` | Goto admission and the `GoalStatus` lifecycle. |
| `.cljc` | `src/pollen_robotics/main.cljk` | The host: CRUD fold, command handlers, and the cores' oracle. |

The split is not cosmetic. What is a **decision** — something that can answer
"no", and whose answer must be the same on every backend — is in Kotoba.
What is **mechanism** — the gRPC round trip, the lidar scan, the clock, the
trajectory, the store — stays on the host. `main.cljc` restates the cores as
an oracle so the host can answer without a WASM round trip, and so the two can
be pinned against each other.


## Units

The SDK presents degrees, metres and seconds as floats. Decisions are made in
**integer millidegrees, millimetres and milliseconds**, converted once at the
host boundary (`deg->millideg`, `m->mm`, `s->ms`).

A limit check that decides whether a motor moves must not depend on which
backend rounded the compare. Millidegrees is finer than any Reachy 2 encoder
reports and millimetres is finer than a lidar resolves, so nothing real is
lost. The conversion rounds half-up, and `fixed-point-conversion` in the test
pins that it does so identically on the JVM and in JS.


## What the cores decide

### `reachy_lidar_safety_core.cljk`

`MobileBaseLidarService` (`protos/mobile_base_lidar.proto`). Classifies a
reading into `LidarObstacleDetectionEnum` and says how much of a commanded
base speed survives it, as a permille scale.

Two gates are exported, and **they deliberately differ**:

- `sdk-speed-scale-permille` — what the robot does. `LidarSafety.safety_on` is
  an operator switch, and with it off the base executes the commanded speed.
- `governed-speed-scale-permille` — what an actor behind a governor may
  command. The switch may relax a `SLOWDOWN`; it may not turn a `STOP`, or an
  unreadable lidar, into motion.

Exporting only the first would make the actor unsafe. Exporting only the
second would make it silently disagree with the robot. Naming both is what
lets a caller state which one it meant, and `handle-safety-gate` returns both.

A negative distance means **no reading**, not "nothing nearby" — the case the
gate exists for. It fails closed through the ordinary path. So does a
misconfigured ring pair (critical ≥ safety, or critical ≤ 0): there is no
reading such a pair can classify correctly, and refusing to answer is the only
safe answer.

### `reachy_goto_core.cljk`

`GoToService` (`protos/goto.proto`), with arm and neck joint goals from
`arm.proto` / `head.proto`.

- **Admission** — part readiness, joint count (7 for an arm, 3 for the neck,
  in the order the enums declare), interpolation mode, duration, joint limits.
  `NONE_INTERPOLATION` is proto3's unset zero value, so it is *refused* rather
  than defaulted: a caller who did not say how to interpolate did not ask for
  minimum jerk. A zero duration is likewise refused — that is the
  `goal_position` path the SDK documents as the dangerous one, not a goto.
- **Lifecycle** — the only legal walk is `ACCEPTED → EXECUTING → {SUCCEEDED,
  ABORTED}`, with `CANCELING` reachable from either live state and settling on
  `CANCELED`. A terminal status is terminal, *including a repeat of itself*, so
  a duplicated completion event cannot be mistaken for progress.

**The core carries no joint limits.** The robot reports its own
(`ArmService.GetJointsLimits`); an actor that hard-codes them asserts a
hardware fact it did not measure, and keeps asserting it after a firmware
change. `within-limit?` takes the reported pair and refuses an inverted or
absent one — so a joint the robot never reported fails closed through the
ordinary path, not through a special case.


## The refusable surface

CRUD over the 12 entities is the generic fold (5 routes each). The SDK's
*commands* are not CRUD, and pretending they were would lose the only thing
that makes them interesting — they can answer no:

| | |
|---|---|
| `POST /v1/gotos/admit` | Decide a goto; persist only if admitted. `409` + the core's reason otherwise. |
| `POST /v1/gotos/{id}/cancel` | `CancelGoTo`. `ack` is true only when the call actually made progress. |
| `POST /v1/gotos/{id}/status` | Advance `GoalStatus`; refuse any walk the lifecycle does not admit. |
| `GET /v1/lidarsafeties/{id}/gate` | Evaluate the gate, reporting **both** scales. |

`409` rather than `400` on a refused goto: the request is well-formed; the
robot's state is what refuses it.


## Tests

```
kbb -M:test     # JVM
npm test            # nbb / ClojureScript
kbb -M:lint     # clj-kondo
```

15 tests, 329 assertions, green on both hosts.

Three different things are checked, and they are not the same thing:

1. **Contract** — the CRUD surface the schema implies.
2. **Behaviour** — the refusable commands, with expected answers written out
   by hand so the corpus is a third opinion rather than a restatement of the
   implementation.
3. **Parity** — `cores/*.cljk` against the oracle in `main.cljc`, by reading
   the core *source* and pinning every wire code and reason literal it
   declares, plus its export set.

### What the parity check is, precisely

(3) is a **constant-and-export parity check, not an execution parity check.**
It catches the drift that actually happens — a reordered proto enum, a renamed
reason, an export that disappeared — and it needs no Kotoba compiler on the
classpath, so it runs in this repo's CI on every push. A constant the core
declares that the test's map does not name fails the test rather than going
unchecked.

### Running the execution parity

Compiling the cores and executing the KIR against this oracle is the stronger
check, and it belongs where the compiler lives. From a `kotoba-lang/kotoba`
checkout, following the pattern of `test/kotoba/cap_use_kotoba_parity_test.clj`:

```clojure
(require '[kotoba.compiler.core :as compiler] '[kotoba.kir :as ir])
(let [kir (:kir (compiler/compile-source (slurp ".../cores/reachy_goto_core.cljk")
                                         :wasm32-kotoba-v1 {}))]
  (ir/execute kir 'admit-code [...]))
```

Neither check replaces the other, and the test file does not claim the one it
is not doing.

### Vocabulary conformance

Both cores are verified against the admitted Kotoba grammar:

```
kbb --backend sci scripts/kotoba-surface-gap.cljk orgs/kotoba-lang/com-pollen-robotics/cores \
  --frontend orgs/kotoba-lang/kotoba-sema/src/kotoba/compiler/frontend.cljc \
  --grammar  orgs/kotoba-lang/kotoba-lang/lang/guest-grammar.edn \
  --stdlib   orgs/kotoba-lang/kotoba-lang/lang/stdlib/core.kotoba
```

`0 distinct` gaps in both directions.


## What is deliberately not here

- **Joint limits, workspace bounds, gear ratios.** The robot reports them.
- **Inverse kinematics.** `ArmService.ComputeArmIK` is mechanism, and a
  clean-room IK that silently disagreed with the robot's would be worse than
  no IK. `EndEffectorPose` stores what the robot solved.
- **The 4×4 pose matrix as 16 stored doubles.** `kinematics.Matrix4x4` carries
  redundancy an EAVT log cannot keep consistent — a 4×4 whose upper block has
  drifted off SO(3) is representable but is not a pose. Translation plus unit
  quaternion is the same information without it; the 4×4 is reconstructed at
  the boundary.
- **Video and audio payloads.** `VideoService.GetFrame` returns bytes; frames
  are not actor state.


## Upstream

| | |
|---|---|
| Vendor | [Pollen Robotics](https://pollen-robotics.com/) |
| Robot | Reachy 2 |
| API definitions | [`pollen-robotics/reachy2-sdk-api`](https://github.com/pollen-robotics/reachy2-sdk-api) — 21 protos, Apache-2.0 |
| Python SDK | [`pollen-robotics/reachy2-sdk`](https://github.com/pollen-robotics/reachy2-sdk) |
| Docs | [docs.pollen-robotics.com](https://docs.pollen-robotics.com/) |
| Verified against | `protos/` @ `main`, 2026-08-28 |

Licensed Apache-2.0, the same licence upstream publishes under.
