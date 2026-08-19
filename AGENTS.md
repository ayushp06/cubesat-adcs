# CubeSat ADCS Project Instructions

This repository is a research-grade CubeSat ADCS digital twin and
flight-software project.

## Before making changes

1. Read `docs/ROADMAP.md`.
2. Read `docs/STATUS.md`.
3. Inspect recent git history.
4. Inspect current git status.
5. Determine the current project phase.
6. Continue from the first incomplete milestone.
7. Do not duplicate completed work.
8. Do not remove validated functionality without justification.

## Primary stack

- MATLAB
- Simulink
- C++
- Python
- CMake
- Linux

## Engineering requirements

- Maintain consistent coordinate/frame/quaternion conventions.
- Use SI units internally.
- Keep the architecture modular, with clear boundaries between configuration,
  mathematics, dynamics, simulation, flight software, and tests.
- Separate truth state from estimated state.
- Validate mathematics analytically where possible.
- Add automated tests for major algorithms.
- Maintain research-grade documentation.
- Explain theory from beginner intuition through implementation.
- Never fabricate test results.

## Git requirements

After every major coherent change:

1. Run relevant tests.
2. Fix regressions.
3. Update documentation.
4. Update `docs/STATUS.md`.
5. Update `docs/ROADMAP.md` if appropriate.
6. Review `git diff`.
7. Commit the change with a descriptive conventional commit.

Do not push unless explicitly instructed.

## When resuming work

1. Read `AGENTS.md`.
2. Read `docs/STATUS.md`.
3. Read `docs/ROADMAP.md`.
4. Run `git log --oneline -15`.
5. Run `git status`.
6. Inspect the latest relevant source and tests.
7. Continue from the current milestone.
