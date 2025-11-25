# ProjectX

This repository contains the ProjectX ROS2 workspace, development container configuration, and CI workflows used by the project. It is no longer a template — the README below explains how to set up the workspace locally or in a devcontainer, how to run CI-related tasks, and how to contribute.

**Prerequisites**
- **Docker**: Required for building the devcontainer and for running the devcontainer CI.
- **Git**: To clone the repo and manage branches.
- **ROS installation (optional for local builds)**: If you want to build locally instead of using the devcontainer, install a ROS distribution and ensure the distro is available under `/opt/ros/<distro>/setup.bash`.
- **Notes about default ROS distro**: This repository references a custom distro name `jazzy` in scripts as the default (see `setup_ws.sh`). If you don't have a `jazzy` ROS installation, set `ROS_DISTRO` to a supported distro you have (for example `humble`) before running local setup commands.

**Quick Local Setup**
- Clone and bootstrap the workspace:

```bash
git clone <your-repo-url> projectX
cd projectX
# If you don't have 'jazzy', set your distro, e.g.:
# export ROS_DISTRO=humble
./setup_ws.sh
```

- What `setup_ws.sh` does:
  - sources `/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash`
  - updates submodules
  - runs `rosdep update` and installs package dependencies
  - runs `colcon build --symlink-install`

- If you prefer to run steps manually:

```bash
export ROS_DISTRO=humble            # or your installed distro
source /opt/ros/${ROS_DISTRO}/setup.bash
git submodule update --init --recursive
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

**Devcontainer (recommended for consistent environment)**
- To open in VS Code: open the repo in VS Code and choose "Reopen in Container" when prompted.
- Devcontainer configs are in the `.devcontainer/` folder. The default config uses the Dockerfile at `.devcontainer/Dockerfile` and a `BASE_IMAGE` build-arg defined in `.devcontainer/default/devcontainer.json`.
- The default `BASE_IMAGE` in this repo references a private registry image (for example `lcas.lincoln.ac.uk/lcas/ros:humble`). If your environment cannot pull that image:
  - Option A — provide registry credentials in GitHub Actions (for CI) or Docker login locally, or
  - Option B — change `BASE_IMAGE` in `.devcontainer/*/devcontainer.json` to a public base image such as `ros:humble` or a public GHCR/ DockerHub image.
- The devcontainer runs a post-create script `.devcontainer/post-create.sh` which attempts to install dependencies and build the workspace; it may continue past failures to compile as much as possible.

**CI (GitHub Actions)**
- CI workflows are under `.github/workflows/`:
  - `ros-ci.yml` — builds and tests the workspace using the ROS CI action. NOTE: this repo currently lists `jazzy` in the workflow matrix. If you want to run CI with a standard distro (e.g., `humble`), edit the matrix in that file.
  - `dev-container.yml` — builds devcontainer images on CI. It pulls the `BASE_IMAGE` referenced by `.devcontainer/*/devcontainer.json`.
- If `dev-container.yml` fails early with pull errors (e.g. `pull access denied`), add repository secrets in GitHub to allow the runner to login to the private registry:
  - `REGISTRY_USERNAME`
  - `REGISTRY_PASSWORD`
  Add them in Settings → Secrets → Actions. The workflow contains a `docker/login-action@v2` step that uses these secrets.

**Build & Test Commands**
- Build the workspace:

```bash
source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash
colcon build --symlink-install
```

- Run tests:

```bash
colcon test --packages-select <package_name>
colcon test-result --verbose
```

**Contributing**
- Add ROS2 packages into the `src/` folder. Create packages using:

```bash
ros2 pkg create --build-type ament_cmake <pkg_name>
```

- Run tests and linters locally (or in the devcontainer) before opening a pull request.

**Troubleshooting & Notes**
- If `setup_ws.sh` or local builds fail due to missing `/opt/ros/jazzy`, either install that distro or set `ROS_DISTRO` to a distro you have (for example `humble`):

```bash
export ROS_DISTRO=humble
```

- If the CI/devcontainer cannot pull a private base image, either provide registry credentials or change the `BASE_IMAGE` in `.devcontainer/*/devcontainer.json` to a public image.

I