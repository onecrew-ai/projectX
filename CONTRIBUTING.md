# Contributing to ProjectX

[![codecov](https://codecov.io/gh/onecrew-ai/projectX/branch/main/graph/badge.svg)](https://codecov.io/gh/onecrew-ai/projectX)

Thank you for contributing. This guide explains the preferred workflow for issues, branches, commits, and pull requests so CI stays healthy and reviews are fast.

### Reporting issues
- Search existing issues before opening a new one.
- Provide: summary, steps to reproduce, expected vs actual behavior, logs (trimmed), and platform/ROS distro.

### Branches and work
- Base work off `main` (or an agreed development branch).
- Use short, descriptive branch names: `feature/<short-desc>`, `bugfix/<short-desc>`, `hotfix/<short-desc>`.

### Commit messages
- Use clear, imperative messages. Example:

```
feat: add joystick teleop to botx-control

Add joystick mapping and parameterized deadzone.

Closes: #123
```

- Prefer small, focused commits. Group related edits in a single PR.

### Pull requests
- Open a PR from your branch into `main` (or target branch).
- In the PR description include: what changed, how to test, and any required environment (ROS distro, hardware, secrets).
- Link relevant issue(s).
- Add reviewers and request CI runs.

### Tests and CI
- Run unit and integration tests locally or in the devcontainer before opening a PR.
- CI workflows are in `.github/workflows/`:
  - `ros-ci.yml` — workspace build/test
  - `dev-container.yml` — builds devcontainer images
- If `dev-container.yml` fails due to private base images, add repository secrets (`REGISTRY_USERNAME`, `REGISTRY_PASSWORD`) in Settings → Secrets → Actions so the workflow can authenticate to pull images.

### Packages as submodules

This repository treats each ROS package as a separate git submodule. New packages should live in their own repositories and be added to this workspace as submodules under `src/`.

Why: keeping packages as submodules allows independent versioning, easier reuse, and smaller pull requests for individual packages.

How to add a new package (recommended flow):

1. Create a new remote repository for the package (e.g. on GitHub/GitLab).
2. Locally, from the ProjectX repo root, add it as a submodule:

```bash
# from project root
git submodule add <repo-url> src/<package_name>
git submodule update --init src/<package_name>
git add .gitmodules src/<package_name>
git commit -m "Add submodule: <package_name>"
git push
```

If you are creating the package locally first and then adding it as a submodule, push the new package repository to its remote before running `git submodule add`.

Updating a package submodule to its latest remote tip:

```bash
cd src/<package_name>
git fetch origin
git checkout main    # or the branch you track
git pull
cd ../..
git add src/<package_name>
git commit -m "Update <package_name> submodule"
git push
```

Removing a package submodule:

```bash
git submodule deinit -f src/<package_name>
git rm -f src/<package_name>
rm -rf .git/modules/src/<package_name>
git commit -m "Remove submodule <package_name>"
git push
```

Include these submodule rules in your PR description when adding/removing packages so reviewers can verify submodule changes.

### Code Style & Linters
- Follow existing style files in the repo:
  - C++: `CPPLINT.cfg` (run `cpplint` if applicable)
  - Python: use `flake8` / `black` (add config if repo requires)
- Run formatting and linter checks locally or in the devcontainer before submitting.

### Review Checklist (for reviewers)
- Does the change have a clear description and test steps?
- Are there tests (unit/integration) or a note explaining why not?
- Does CI pass or are failures unrelated to the change?
- Are secrets/credentials handled via repository secrets (not hardcoded)?

## Coverage Badges

We publish coverage badges from Codecov to indicate project and per-package coverage.

- Project badge (example at top):

```
[![codecov](https://codecov.io/gh/onecrew-ai/projectX/branch/main/graph/badge.svg)](https://codecov.io/gh/onecrew-ai/projectX)
```

- Per-package badge (replace `<pkg_name>` and `branch` as needed):

```
[![codecov](https://codecov.io/gh/onecrew-ai/projectX/branch/main/graph/badge.svg?flag=<pkg_name>)](https://codecov.io/gh/onecrew-ai/projectX)
```

To create a per-package badge in Codecov:
1. Upload coverage with a flag (we use the package name as the flag in CI uploads).
2. Open the Codecov UI for this repo: `Settings → Repository → Badges`.
3. Select the branch and flag, copy the generated markdown, and paste it into the README or `CONTRIBUTING.md`.

### Quick commands

```bash
# build and test locally (example)
export ROS_DISTRO=humble
source /opt/ros/${ROS_DISTRO}/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
colcon test
colcon test-result --verbose
```

Thanks for contributing — your improvements make ProjectX better.
