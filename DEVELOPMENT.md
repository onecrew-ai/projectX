# Development Guide

This document contains step-by-step instructions for setting up a development environment, using the devcontainer, building, testing, and debugging ProjectX.

## Recommended workflow
- Use the devcontainer for a reproducible environment. If you must work locally, ensure you have a compatible ROS distro and dependencies installed.

## Devcontainer (VS Code)
1. Open the repository in VS Code.
2. When prompted choose "Reopen in Container". If not prompted, run: `F1` → `Remote-Containers: Reopen in Container`.
3. The devcontainer uses `.devcontainer/Dockerfile` and `.devcontainer/*/devcontainer.json`.
4. The devcontainer runs `.devcontainer/post-create.sh` which tries to install dependencies and build the workspace. This may take several minutes.

Notes:
- The default `BASE_IMAGE` set in `.devcontainer/*/devcontainer.json` currently references a private registry (for example `lcas.lincoln.ac.uk/lcas/ros:humble`). If you cannot pull that image locally, change the `BASE_IMAGE` build-arg to a public image (for example `ros:humble`) or log in to the registry.

## Local setup (without devcontainer)
1. Ensure you have a ROS distro installed and available at `/opt/ros/<distro>`.
2. From the repo root:

```bash
git submodule update --init --recursive
export ROS_DISTRO=humble  # or the distro you have
source /opt/ros/${ROS_DISTRO}/setup.bash
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

## Building & testing
- Build: `colcon build --symlink-install`
- Run tests: `colcon test` then `colcon test-result --verbose`
- Build a single package: `colcon build --packages-select <pkg_name>`

## Running nodes and launch files
- Source the workspace before running nodes:

```bash
source install/setup.bash
ros2 run <package> <executable>
ros2 launch <package> <launch_file.launch.py>
```

## Debugging
- Use VS Code remote debugger (attach to process inside the devcontainer) or print/log statements.
- For C++ set breakpoints in VS Code and launch via the `launch.json` in `.vscode` (if present) or create one.

## CI & Devcontainer CI quick notes
- `ros-ci.yml` runs workspace build/test. Check the matrix distro in that file (`jazzy` by default here).
- `dev-container.yml` builds devcontainer images. The workflow expects to be able to pull `BASE_IMAGE`. If images are private, add secrets `REGISTRY_USERNAME` and `REGISTRY_PASSWORD` in the repo (Settings → Secrets → Actions).

## Useful commands

```bash
# Launch an interactive shell in the devcontainer image (locally)
docker build --build-arg BASE_IMAGE=ros:humble -f .devcontainer/Dockerfile -t projectx-dev .
docker run --rm -it projectx-dev bash

# Quick check of setup script syntax
bash -n setup_ws.sh

# Lint, format and tests (example)
# run any project-specific linters configured in the repo
```

## When things fail
- If `rosdep install` fails: check package.xml for invalid dependencies and internet access in container/host.
- If `colcon build` fails: run `colcon build --event-handlers console_direct+` to see full output for failing packages.
- If devcontainer build fails due to pull access: login to the registry or use a public base image.

## Packages as submodules

ProjectX keeps each ROS package as a separate git submodule under `src/`. This allows independent package versioning and smaller package-specific PRs.

Common workflows:

- Add a new package (after creating its remote repo):

```bash
# from project root
git submodule add <repo-url> src/<package_name>
git submodule update --init src/<package_name>
git add .gitmodules src/<package_name>
git commit -m "Add submodule: <package_name>"
git push
```

- Clone this repo including submodules:

```bash
git clone --recurse-submodules <repo-url>
# or after clone:
git submodule update --init --recursive
```

- Update all submodules to their latest tracked commits:

```bash
git submodule update --remote --merge
```

- Remove a submodule:

```bash
git submodule deinit -f src/<package_name>
git rm -f src/<package_name>
rm -rf .git/modules/src/<package_name>
git commit -m "Remove submodule <package_name>"
git push
```

If you add or remove submodules, remember to mention `.gitmodules` changes in the PR and include the submodule path in the commit so CI and reviewers can validate the new package.

