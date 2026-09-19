# Criticality Rationale — `feature_tracking`

## Up front

`lunar_simulator` is a Gazebo rover simulator running on a developer's own
machine. It does not fly, does not control real hardware, and no failure of
this code can hurt a person, damage equipment, or lose a mission. ECSS-Q-ST-80C
("Software product assurance") is a standard for space-project software
suppliers and their customers; this project has neither. Reproducing its
categorization scheme here is a **rigor exercise** — using space-software
discipline as a stretch target for how well-engineered a simulator's
localisation code can be — not a claim that this project sits inside ECSS's
actual scope of application.

## ECSS-Q-ST-80C Annex D criticality categories (Table D-1)

Reproduced from the standard (`ECSS-Q-ST-80C-Rev.2.md` Annex D.1, Table D-1,
as held at `~/personal/ArvSimulator/Docs/CodingStandards/` on this machine —
an unrelated project's copy of the genuine ECSS text, not wired to any
`.claude` skill):

| Category | Definition |
|---|---|
| A | Software involved in category I functions with no compensating provisions, or software included in compensating provisions for category I functions. |
| B | Software involved in category I functions **with** at least one compensating provision available (a hardware implementation; a software implementation itself classified category A; or an operational procedure), or software involved in category II functions with no compensating provisions. |
| C | Software involved in category II functions **with** at least one compensating provision (hardware implementation; a software implementation itself classified category B; or an operational procedure), or software involved in category III functions with no compensating provisions. |
| D | Software involved in category III functions **with** at least one compensating provision (hardware implementation; a software implementation itself classified category C; or an operational procedure), or software involved in category IV functions with no compensating provisions. |

Categories I–IV (the underlying *function* criticality these categories are
built from) come from ECSS-Q-ST-30/ECSS-Q-ST-40, both outside this
repository's available reference material — reproducing their exact text
here would be a guess, so this document does not attempt it.

## Category: intentionally unassigned

**No criticality category (A/B/C/D) is assigned to `feature_tracking` in this
project.** This is the user's explicit, confirmed choice during planning —
not a gap the agent failed to fill in, and not a default. Assigning a real
ECSS category requires classifying the *function* this code serves
(per ECSS-Q-ST-30/40, unavailable here) and an authority empowered to make
that classification for a real program; neither exists for a personal
simulator. Recording "unassigned" honestly, as an open item, was judged more
useful than picking a plausible-sounding category with no one accountable
for having chosen it.

## What Table D-2 would tighten or loosen, and why the chosen rigor doesn't
wait for a category

ECSS-Q-ST-80C Annex D.2's Table D-2 tailors *every* clause of the standard
per category (Y = applicable, N = not applicable, or conditional). Three
clauses illustrate the range, verified directly against the standard text:

- **§6.2.3.2** (critical-software measures) lists a *menu* of optional
  measures for critical software — "100 % code branch coverage at unit
  testing level" is one bullet among several (others: full source
  inspection, witnessed/independent testing, a "safe subset" of the
  language, defensive programming). **This is not a blanket 100%-coverage
  mandate** even for the most critical category — it is one option a
  supplier may choose among several to satisfy the parent requirement
  ("define, justify and apply measures to assure dependability and safety
  of critical software"). A plan or agent that read this clause as "ECSS
  requires 100% branch coverage" would be citing it wrong; this project's
  design corrects that reading rather than repeating it.
- **§6.2.3.7 / §6.2.3.8** require unit/integration and validation testing
  to be *(re-)executed on non-instrumented code* — i.e. the code path
  actually shipped, not a specially-instrumented build. This project's own
  tests (`test_corner_detector.cpp`, `test_optical_flow_tracker.cpp`) do
  run against the same `alpha_feature_tracking` static library the ROS
  node links, built with the same flags — satisfying the *spirit* of this
  clause even with no category assigned.
- **§6.3.5.2** requires that "test coverage goals for each testing level
  shall be agreed between the customer and the supplier" and their
  achievement monitored — a negotiated goal, not a standard's own fixed
  number, and precisely why this project states its test coverage as a
  named list of behaviors in `TRACEABILITY_MATRIX.md` rather than inventing
  a percentage nobody agreed to.

Category A/B software would additionally expect things this project does
not have: independent (not self-) verification of the implementation
against requirements, a customer-agreed test plan, and formal
configuration management with controlled baselines. This project has none
of those — no customer, no independent reviewer, one developer session.
The chosen engineering rigor (deterministic algorithms, two-phase init, no
heap after init, `noexcept` throughout, full clang-tidy triage, a named
test list) approximates what a **B/C-equivalent** engineering discipline
would look like, per the user's explicit "don't skip things" instruction —
but doing that work is not the same as *being* category B or C, which also
requires the process controls (independent review, agreed test plan,
formal CM) this single-session project cannot claim.

## Restated

Assigning a category later is a project decision, not a technical one this
document can make for the project owner. If a category is ever assigned,
revisit this document, `JSF_AV_APPLICABILITY_PROFILE.md` §1 and §8, and
`DEVIATION_LOG.md`'s approval fields together — a real category changes
what verification evidence is actually required, not just what this
document says about it.
