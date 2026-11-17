# Implementation plan: Update dependencies

## Task summary

Bump the six `com.irurueta` compile-scope dependencies in `pom.xml` to the latest patch versions listed in the issue
(`irurueta-numerical` 1.6.1, `irurueta-geometry` 1.6.1, `irurueta-units` 1.4.1, `irurueta-algebra` 1.4.1,
`irurueta-navigation` 1.8.2, `irurueta-navigation-inertial` 1.10.2), then verify the project still builds and passes
its tests and quality gates. Approach: edit versions in place (no property extraction) to match the existing
`pom.xml` style; all bumps are patch-level, so no source changes are expected.

Source: GitHub issue #23
Base branch: develop
Open clarifications: none

## Acceptance criteria coverage

| # | Criterion | Covered by tasks |
|---|---|---|
| 1 | "Update dependencies in pom.xml file to latest versions of: irurueta-numerical 1.6.1, irurueta-geometry 1.6.1, irurueta-units 1.4.1, irurueta-algebra 1.4.1, irurueta-navigation 1.8.2, irurueta-navigation-inertial 1.10.2" | Task 1 |
| 2 | derived: project compiles and all existing tests pass against the new versions | Task 2 |
| 3 | derived: changelog records the dependency update | Task 3 |

## Current code state

- Project type: `java-library` (Maven, no wrapper, single module, version `1.9.0-SNAPSHOT`).
- `pom.xml` lines ~60-95 declare the six dependencies at: numerical 1.6.0, geometry 1.6.0, units 1.4.0,
  algebra 1.4.0, navigation 1.8.1, navigation-inertial 1.10.1.
- No README/Antora page references these dependency versions; `CHANGELOG.md` follows Keep a Changelog.
- No `.archive/` precedent exists.

## Implementation steps

### Group 1 — Bump dependencies (Parallelizable: yes)

- [x] Task 1. Update dependency versions in `pom.xml` _(java)_ — pom.xml only; full suite passes after bump (no source or test changes, coverage unchanged; no new quality issues possible as no Java touched). License/Javadoc not applicable.
  - [x] Task 1.1. Set `irurueta-numerical` and `irurueta-geometry` to `1.6.1`
  - [x] Task 1.2. Set `irurueta-units` and `irurueta-algebra` to `1.4.1`
  - [x] Task 1.3. Set `irurueta-navigation` to `1.8.2` and `irurueta-navigation-inertial` to `1.10.2`

### Group 2 — Verify and document (Parallelizable: no — Task 2 needs Task 1 landed; Task 3 is ordered after verification)

- [x] Task 2. Build and verify against the new versions _(java)_ — no source/test changes; 662 tests pass (0 failures, 6 skipped); coverage 93.68% line / 87.25% branch; Checkstyle 2735, PMD 88, SpotBugs 210 identical to baseline (no regressions). License/Javadoc not applicable.
  - [x] Task 2.1. Run the full test suite via the `iru-gate-runner` agent (`iru-java-test`); fix any compile or
    test failures caused by the new library versions
  - [x] Task 2.2. Run `iru-java-coverage` and `iru-java-code-quality` via `iru-gate-runner`; confirm no regressions
    versus baseline
- [x] Task 3. Add a "Changed" entry for the dependency bump under the unreleased section of `CHANGELOG.md` — CHANGELOG.md only.
