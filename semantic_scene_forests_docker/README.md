# Semantic Scene Forests Docker Setup

Quick start guide for the semantic scene forests Docker deployment.

This folder lives inside the semantic scene forests monorepo and builds the local working copy
of every `semantic_scene_forests_*` package. It is `COLCON_IGNORE`'d so colcon never treats it as a ROS
package. The build context for both images is the **monorepo root** (`../..` from the
`robots/` and `opc/` compose stacks), and the Dockerfiles `COPY` the local source straight
into the image — only external/third-party dependencies are still pulled via `vcs import`
(`robots_repos.yaml`).

Two images are produced:

- **`robot`** — headless deployment image; the `semantic_scene_forests_visualization_rviz` and
  `semantic_scene_forests_visualization_rqt` packages (and their rviz/rqt/Qt/Ogre deps) are excluded.
- **`opc`** — extends `robot`, adds GUI tooling (X11/Wayland, `rviz2`, `rqt`) and builds the
  two visualization plugins. Used for visualization and as a devcontainer.

All commands below assume you start from this `semantic_scene_forests_docker/` folder.

## Quick Start

### 1. Prerequisites

```bash
# Install NVIDIA Container Toolkit (for GPU support)
sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker
```

### 2. Build Images

The build needs SSH agent forwarding (for the private external repos); the compose files
already declare `ssh: default`, so just have your key loaded in `ssh-agent`.

The compose files also accept optional image overrides via `SSF_ROBOT_IMAGE` and
`SSF_OPC_IMAGE`. Local workflows can ignore them; CI uses them to publish commit- and
branch-tagged images without editing the compose files.

```bash
# Build the headless deployment (robot) image first
cd robots
DOCKER_BUILDKIT=1 docker compose build

# Build the OPC image (extends the robot image, adds rviz/rqt)
cd ../opc
DOCKER_BUILDKIT=1 docker compose build
```

### 3. Run Services

**Robots Stack:**
```bash
cd robots
docker compose up -d
```

**OPC Stack:**
```bash
cd opc
docker compose up -d
```

### 4. Configure Zenoh

Edit `robots/ros_config.env` and `opc/ros_config.env` to fill in:
```bash
ZENOH_ROUTER_CONFIG_URI=/zenoh_configs/path/to/router_config.json5
ZENOH_SESSION_CONFIG_URI=/zenoh_configs/path/to/session_config.json5
```

### 5. Access Development Environment

```bash
cd opc
docker compose exec opc_devcontainer zsh
source /home/ubuntu/ws/install/setup.bash
```

## Directory Structure

```
semantic_scene_forests/        # monorepo root = Docker build context
├── semantic_scene_forests_docker/  # this folder (COLCON_IGNORE'd)
│   ├── COLCON_IGNORE
│   ├── docker/
│   │   ├── Dockerfile.robots  # robot image (headless perception stack)
│   │   ├── Dockerfile.opc     # opc image (extends robot, adds rviz/rqt GUI)
│   │   ├── additional-apt-requirements.txt
│   │   └── additional-pip-requirements.txt
│   ├── robots/                # deployment compose stack
│   │   ├── compose.yaml
│   │   ├── common.yaml        # context: ../..  dockerfile: semantic_scene_forests_docker/docker/Dockerfile.robots
│   │   ├── mounts/{dev_ws/src/, additional_launch_files/pipeline.toml}
│   │   ├── zenoh-configs/
│   │   └── ros_config.env
│   ├── opc/                   # visualization / devcontainer compose stack
│   │   ├── compose.yaml
│   │   ├── common.yaml
│   │   ├── mounts/dev_ws/src/
│   │   ├── zenoh-configs/
│   │   └── ros_config.env
│   ├── robots_repos.yaml      # external dependency repos only
│   └── opc_repos.yaml         # placeholder (unused — viz built from local source)
├── semantic_scene_forests/ semantic_scene_forests_msgs/ ...   # the ROS packages (COPY'd into the image)
└── .dockerignore
```

See `AGENTS.md` for the original/full design documentation.

## Services

**robots (deployment):**
- `zenoh` - DDS router
- `semantic_scene_forests` - core perception pipeline via `scene_graph_combined.launch.xml`
  (object tracker → object manager → generation → query in one component container)

**opc (visualization / dev):**
- `zenoh` - DDS router
- `semantic_scene_forests_visualization` - runs `rviz2` hosting the `semantic_scene_forests_visualization_rviz` plugin
- `opc_devcontainer` - interactive dev shell (rviz2 + rqt available)

## Docker Images

**robot image**
(`…/semantic_scene_forests_generation/semantic_scene_forests/robot:kilted`):
- Headless perception stack, all packages built from the local monorepo
- Excludes the rviz/rqt visualization packages and their GUI dependencies
- Builds as the `ubuntu` user to avoid permission issues

**opc image**
(`…/semantic_scene_forests_generation/semantic_scene_forests/opc:kilted`):
- Extends the robot image
- Adds GUI dependencies (X11, Wayland) and the `rviz2`/`rqt` apps
- Builds the `semantic_scene_forests_visualization_rviz` and `semantic_scene_forests_visualization_rqt` plugins

## Common Commands

```bash
# View logs
docker compose logs -f semantic_scene_forests

# Restart a service
docker compose restart semantic_scene_forests

# Monitor services
docker compose ps

# View resource usage
docker stats
```

## GPU Support

GPU is enabled by default. To disable, comment out `deploy: *gpu-deploy` in compose.yaml files.

## GUI Support (OPC)

Enable X11 forwarding:
```bash
xhost +local:docker
```

## CI/CD Pipeline

The Docker images can be built and pushed from the monorepo GitLab CI. The root
`.gitlab-ci.yml` now contains the two Docker jobs alongside the shared ROS 2 pipeline include.

### Pipeline Overview

Two jobs, run from this folder:
1. **build_robots**: builds and pushes the `robot` image (`cd semantic_scene_forests_docker/robots && docker compose build && docker compose push`)
2. **build_opc**: builds and pushes the `opc` image, depends on `build_robots`
   (`cd semantic_scene_forests_docker/opc && docker compose pull || true && docker compose build && docker compose push`)

Images are tagged with:
- Commit SHA (`$CI_COMMIT_SHA`)
- Branch name (`$CI_COMMIT_REF_SLUG`)
- `latest` (for main branch)

### When Pipeline Runs

- On pushes to `main` branch
- On pushes to `release/*` branches
- On merge requests (uses short SHA for tags)

### Setup Required

#### 1. SSH Private Key for Repository Access

The Docker builds need SSH access to clone repositories from `ids-git.fzi.de`. Add an SSH key as a CI/CD variable:

1. Generate or use an existing SSH key with access to all required repositories
2. Go to **Settings > CI/CD > Variables** in GitLab
3. Add a new variable:
   - **Key**: `SSH_PRIVATE_KEY`
   - **Value**: Paste the private key content (including `-----BEGIN OPENSSH PRIVATE KEY-----` and `-----END OPENSSH PRIVATE KEY-----`)
   - **Type**: Variable
   - **Protected**: ✅ Yes (if only using protected branches)
   - **Masked**: ❌ No

#### 2. Registry Authentication

The pipeline uses GitLab's built-in registry authentication via:
- `$CI_REGISTRY_USER` - Your GitLab username
- `$CI_REGISTRY_PASSWORD` - Your GitLab password or deploy token

**Option A: Using CI_JOB_TOKEN (Recommended)**

The pipeline automatically uses `$CI_REGISTRY_PASSWORD` which is available in all pipelines.

**Option B: Using a Deploy Token (for external pipelines)**

If you need to run this from an external CI system:

1. Go to **Settings > Repository > Deploy Tokens**
2. Add a deploy token:
   - **Name**: `docker-push-token`
   - **Username**: Will be auto-generated
   - **Scopes**: ✅ `read_registry`, ✅ `write_registry`
   - **Expires at**: Optional
3. Copy the generated token
4. Add as CI/CD variables:
   - **Key**: `CI_REGISTRY_USER` (the username from deploy token)
   - **Key**: `CI_REGISTRY_PASSWORD` (the token value)

#### 3. Runner Requirements

The pipeline requires Docker runners with:
- Docker-in-Docker (dind) service
- NVIDIA GPU support (tagged with `nvidia`)

Configure your runner in `/etc/gitlab-runner/config.toml`:
```toml
[[runners]]
  name = "docker-nvidia-runner"
  url = "https://ids-git.fzi.de/"
  token = "YOUR_RUNNER_TOKEN"
  executor = "docker"
  [runners.docker]
    tls_verify = false
    image = "docker:24.0"
    privileged = true
    shm_size = 0
    volumes = ["/var/run/docker.sock:/var/run/docker.sock", "/cache"]
  [runners.cache]
    [runners.cache.s3]
  [runners.docker.services]
    name = "docker:24.0-dind"
    alias = "docker"
```

### Manual Trigger

You can manually trigger the pipeline from GitLab:
1. Go to **Build > Pipeline**
2. Click **Run pipeline**
3. Select branch and click **Run pipeline**

### Image Tags

After a successful pipeline run, images will be available at:
- robot: `ids-git.fzi.de:5555/interne-projekte/dissertations/oberacker_semantic_multi_robot_mapping/semantic_scene_forests_generation/semantic_scene_forests/robot:<tag>`
- opc: `ids-git.fzi.de:5555/interne-projekte/dissertations/oberacker_semantic_multi_robot_mapping/semantic_scene_forests_generation/semantic_scene_forests/opc:<tag>`

Example pull commands:
```bash
docker pull ids-git.fzi.de:5555/interne-projekte/dissertations/oberacker_semantic_multi_robot_mapping/semantic_scene_forests_generation/semantic_scene_forests/robot:latest
docker pull ids-git.fzi.de:5555/interne-projekte/dissertations/oberacker_semantic_multi_robot_mapping/semantic_scene_forests_generation/semantic_scene_forests/opc:latest
```
