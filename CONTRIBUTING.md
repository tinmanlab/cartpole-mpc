# Contributing

Keep the repository as one small teaching system, not a collection of unrelated controller demos.

Before opening a change:

    npm ci
    npm run check

If Playwright is available:

    python3 tests/test_browser.py

Rules:
- keep one authoritative nonlinear plant,
- do not give each lesson its own hidden dynamics,
- label exact implementations, structural analogues, and paper-only concepts separately,
- do not promote CartPole results to humanoid or hardware claims,
- preserve the shared live simulation and graph contract across topic pages,
- add a new abstraction only when an existing path cannot express the required experiment.

Regenerate the checked-in standalone page after source edits:

    python3 scripts/build.py

README demo media can be regenerated with:

    python3 scripts/record_demo.py
