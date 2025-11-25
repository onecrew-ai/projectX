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

- For devcontainer CI failures, paste the failing job log (the step that errors) and I can diagnose the exact cause.

---

If you want, I can:
- switch the README to explicitly document `humble` as the recommended distro and change `setup_ws.sh` defaults,
- update `.devcontainer/*` to use a public `ros:humble` base image, or
- add a short `CONTRIBUTING.md` and `DEVELOPMENT.md` with expanded instructions.

Tell me which of these you'd like next.
## ROS2 Package Template

This repository serves as a template for creating ROS2 packages, equipped with a comprehensive CI workflow and devcontainer configuration.

### Development Environment Setup

To begin development in a containerized environment:

1. **Use this repo as a template:**
   The best way to work with this repo is to use it as a template for your ROS package development, to do so, in the top right corner select `Use this template`:
   
   ![2024-04-24](https://github.com/LCAS/ros2_pkg_template/assets/47870260/2aba3511-7a3f-4e88-a3c1-26ba2be48b45)

   Then in the next step specify the owner and the package name as shown below:
      

3. **Open in Visual Studio Code:**
   Open the cloned repository in VSCode. VSCode will prompt you to "Reopen in Container." Alternatively, you can use the command palette (`Ctrl+Shift+P`) and search for the "reopen in container" command.

   ![Reopen in Container](https://github.com/LCAS/ros2_pkg_template/assets/47870260/52b26ae9-ffe9-4e7c-afb9-88cee88f870f)

   Then this will promote you with the following two options:
   ![image](https://github.com/user-attachments/assets/d0885c75-59de-4b5d-a8b7-c38bf02444d4)

   You may select the base image according to your targeted application. For instance, if the nodes do not require GPU processing tasks, it is preferable to use the default devcontainer as it is more lightweight.

5. **Container Setup:**
   Once reopened in the container, VSCode will initiate the building process and pull all necessary dependencies. You can monitor the building log within VSCode.

   ![Devcontainer Log](https://github.com/LCAS/ros2_pkg_template/assets/47870260/4a01e140-972e-4f10-b866-acaabf6b4cfd)

6. **Verify Container Environment:**
   After the build completes, VSCode will connect to the container. You can verify that you are within the container environment.

   ![In Container](https://github.com/LCAS/ros2_pkg_template/assets/47870260/9efec878-5d83-4aed-a9d0-8a1cf6bbf655)

### Devcontainer Features

The devcontainer includes a light desktop interface. To utilize this feature:

1. **Configuration:**
   Add the following features to the devcontainer configuration:

   ```json
   "features": {
       "ghcr.io/LCAS/devcontainer-features/desktop-lite:1": {}
   },
   "forwardPorts": [6080, 5801],
   "portsAttributes": {
       "6080": {
           "label": "desktop"
       },
       "5801": {
           "label": "desktop opengl"
       }
   }
   ```

2. **Accessing the Desktop Interface:**
   Open the user interface by navigating to the PORTS tab in VSCode, selecting port `6080` (or port `5801` for the CUDA-OpenGL version), and opening it in the browser.

   ![Open in Browser](https://github.com/LCAS/ros2_pkg_template/assets/47870260/b61f4c95-453b-4c92-ad66-5133c91abb05)

3. **Connecting to the Interface:**
   Click on "Connect" and use the password `vscode` to access the desktop interface.

   ![NoVNC](https://github.com/LCAS/ros2_pkg_template/assets/47870260/71246a4c-fd02-4196-b390-b18804f9cd4e)

### Enjoy Development!

By leveraging this setup, you can develop on a remote machine with a lightweight desktop interface. Magic! Furthermore, this template package provides very nice ROS2 functionality like syntax highlight and template code generation. 

**All ROS2 packages should go into the `src/` folder. Create them with `ros2 pkg create...`.**

**The devcontainer tries to install all dependencies of the workspace automatically as much as possible, and also tries to build the workspace when it is created, to speed up later colcon builds.**

### References

1. [ros2-teaching-ws](https://github.com/LCAS/ros2-teaching-ws)
2. [Get Started with Dev Containers in VS Code](https://youtu.be/b1RavPr_878?si=ADepc_VocOHTXP55)
