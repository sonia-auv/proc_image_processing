# proc_image_processing

![Docker Image CI - Master Branch](https://github.com/sonia-auv/proc_image_processing/workflows/Docker%20Image%20CI%20-%20Master%20Branch/badge.svg)
![Docker Image CI - Develop Branch](https://github.com/sonia-auv/proc_image_processing/workflows/Docker%20Image%20CI%20-%20Develop%20Branch/badge.svg?branch=develop)
![GitHub release (latest by date)](https://img.shields.io/github/v/release/sonia-auv/proc_image_processing)
![Average time to resolve an issue](https://isitmaintained.com/badge/resolution/sonia-auv/proc_image_processing.svg)

[![Maintainability Rating](https://sonarcloud.io/api/project_badges/measure?project=sonia-auv_proc_image_processing&metric=sqale_rating)](https://sonarcloud.io/dashboard?id=sonia-auv_proc_image_processing)
[![Reliability Rating](https://sonarcloud.io/api/project_badges/measure?project=sonia-auv_proc_image_processing&metric=reliability_rating)](https://sonarcloud.io/dashboard?id=sonia-auv_proc_image_processing)
[![Security Rating](https://sonarcloud.io/api/project_badges/measure?project=sonia-auv_proc_image_processing&metric=security_rating)](https://sonarcloud.io/dashboard?id=sonia-auv_proc_image_processing)

[![Bugs](https://sonarcloud.io/api/project_badges/measure?project=sonia-auv_proc_image_processing&metric=bugs)](https://sonarcloud.io/dashboard?id=sonia-auv_proc_image_processing)
[![Code Smells](https://sonarcloud.io/api/project_badges/measure?project=sonia-auv_proc_image_processing&metric=code_smells)](https://sonarcloud.io/dashboard?id=sonia-auv_proc_image_processing)
[![Coverage](https://sonarcloud.io/api/project_badges/measure?project=sonia-auv_proc_image_processing&metric=coverage)](https://sonarcloud.io/dashboard?id=sonia-auv_proc_image_processing)
[![Duplicated Lines (%)](https://sonarcloud.io/api/project_badges/measure?project=sonia-auv_proc_image_processing&metric=duplicated_lines_density)](https://sonarcloud.io/dashboard?id=sonia-auv_proc_image_processing)
[![Lines of Code](https://sonarcloud.io/api/project_badges/measure?project=sonia-auv_proc_image_processing&metric=ncloc)](https://sonarcloud.io/dashboard?id=sonia-auv_proc_image_processing)
[![Technical Debt](https://sonarcloud.io/api/project_badges/measure?project=sonia-auv_proc_image_processing&metric=sqale_index)](https://sonarcloud.io/dashboard?id=sonia-auv_proc_image_processing)
[![Vulnerabilities](https://sonarcloud.io/api/project_badges/measure?project=sonia-auv_proc_image_processing&metric=vulnerabilities)](https://sonarcloud.io/dashboard?id=sonia-auv_proc_image_processing)
---

## Getting Started

Clone current project by using following command :

```bash
    git clone git@github.com:sonia-auv/proc_image_processing.git
```

These instructions will get you a copy of the project up and running on your local machine for development and testing
purposes. See deployment for notes on how to deploy the project on a live system.
---

### Prerequisites

#### Docker

First and foremost to run the module you will need to
have [docker](https://www.docker.com/get-started?utm_source=google&utm_medium=cpc&utm_campaign=getstarted&utm_content=sitelink&utm_term=getstarted&utm_budget=growth&gclid=CjwKCAjw57b3BRBlEiwA1Imytuv9VRFX5Z0INBaD3JJNSUmadgQh7ZYWTw_r-yFn2S4XjZTsLbNnnBoCPsIQAvD_BwE)
installed.

To validate your installation of docker, simply type in `docker -v`

If you receive an output in the likes of :

```
Docker version 19.03.5, build 633a0ea
```

It means you have it installed. If not, follow instructions on how to install it for your OS.

#### Docker-Compose

In order to use provided Docker-Compose files, you must have the latest version of Docker-Compose installed, or it must
support version `3.9` of compose files.

See https://docs.docker.com/compose/install/ for installation.

---

### Project Modes

This project has to variants, or modes, that can be built.

This is mainly due to the fact that we have both an implementation using:

- OpenCV **without CUDA** support (`CPU Mode`)
- OpenCV **with CUDA** support (`GPU Mode`)

If a GPU is detected, the NVIDIA Tools present and OpenCV is compiled with CUDA, then the `GPU Mode` is used. Otherwise,
the project is built in `CPU Mode`.

#### GPU Mode requirements

In order to use GPU mode, you need:

- [nvidia-container-runtime](https://github.com/NVIDIA/nvidia-container-runtime)
  or [nvidia-docker2](https://github.com/NVIDIA/nvidia-docker).
- A compatible GPU: Maxwell generation and up.
- A compatible NVIDIA driver installed.

**Support for GPU mode with Windows has not been tested and is not recommended.**

---

### Environment configuration

First, you must be logged on the GitHub Docker Registry.

In order to do this:

- Generate a personal access token on
  GitHub ([Settings | Developper Settings | Personal acces tokens](https://github.com/settings/tokens/new)) with at
  least the scope `read:packages`.
- Type `docker login ghcr.io -u YOUR_GITHUB_USERNAME` and use your generated access token as password.

- Pull sonia_common image:
    - CPU mode: `docker pull ghcr.io/sonia-auv/sonia_common/sonia_common:x86-perception-latest`.
    - GPU mode: `docker pull ghcr.io/sonia-auv/sonia_common/sonia_common_cuda:x86-perception-latest`.

---

#### VSCode

TODO document how to run this module with VSCode

- Install these extensions:
    - C/C++ (extension id: `ms-vscode.cpptools`)
    - Docker (extension id: `ms-azuretools.vscode-docker`)
    - Python (extension id: `ms-python.python`)
    - Remote - Containers (extension id: `ms-vscode-remote.remote-containers`)
    - ROS (extension id: `ms-iot.vscode-ros`)


- Setup `devcontainer` extension:
    - CPU Mode: Copy `.devcontainer/devcontainer-cpu.json` to `.devcontainer/devcontainer.json`
    - GPU Mode: Copy `.devcontainer/devcontainer-gpu.json` to `.devcontainer/devcontainer.json`


- Open the project on a remote container:
    - `ctrl+shift+p`
    - Type `Remote-Containers: Reopen in Container` (command id: `remote-containers.reopenInContainer`)


- Start ROS core:
    - `ctrl+shift+p`
    - Type `ROS: Start Core` (command id: `ros.startCore`)

---

#### CLion

CLion provides more features than VSCode and is typically more powerful in several use case.

To get your environment setup with it, follow these steps:

- Copy `.env.example` to `.env` and change the exposed SSH port that will be used to connect to the CLion remote
  environment with the `CLION_SSH_PORT` variable. You can use the default value of `2222`.


- Build the remote environment Docker image:
    - CPU mode: `docker-compose build proc_image_processing clion_remote_env`.
    - GPU mode: `docker-compose -f docker-compose-gpu.yml build proc_image_processing clion_remote_env`.


- Launch the remote environment:
    - CPU mode: `docker-compose up clion_remote_env`.
    - GPU mode: `docker-compose -f docker-compose-gpu.yml up clion_remote_env`.
    - If this error happens: ` standard_init_linux.go:228: exec user process caused: no such file or directory`, you are
      probably using a Windows host, and you need to make sur end of lines for `sonia_entrypoint.sh`
      and `sonia_clion_entrypoint.sh` are `LF`.
- Configure your Toolchain:
    - Go to `File | Settings | Build, Execution, Deployment | Toolchains`.
    - Add a new `Remote Host` Toolchain:
        - Give it a name, like `Remote Environment`.
        - Configure the credentials by clicking on the kog icon:
            - Give it a name, like `Remote Environment`.
            - Use `127.0.0.1` as `Host` value.
            - Use the value that you used for the `CLION_SSH_PORT` variable as `Port` value.
            - Use `sonia` as `User name` value.
            - Use `test` as `Password` value.
            - Leave the remaining fields with their default values.
            - Try using the `Test connection` button to make sure CLion can SSH into your remote environment.
        - Once you are done with the credentials, CLion should find CMake, Make, the C Compiler, the C++ Compiler and
          the Debugger.


- Configure CMake:
    - Go to `File | Settings | Build, Execution, Deployment | CMake`.
    - Add a new profile:
        - Use `Debug` as the `Build type` value.
        - Use the toolchain name you just configured as the `Toolchain` value.
        - Use `build` as the `Build directory` value.
        - Hit `Apply` button (you will need to close the window in order to do the next step).
        - Copy and paste the contents from the [clion.env.vars](clion.env.vars) file as the `Environment` value.
        - Make sure that `Include system environment variables` is checked when you click on the small icon at the end
          of the `Environment` field.


- Configure Deployment:
    - Go to `File | Settings | Build, Execution, Deployment | Deployment`.
    - Modify `SFTP` deployment that has been automatically created by the previous steps:
        - Give it a name, like `Remote Environment`.
        - Use your configured credentials as `SSH configuration`. Test that the connection still works!
        - Go to the `Mappings` tab:
            - Use the path of proc_image_processing as `Local path` value. For
              example: `/home/yourusername/CLionProjects/proc_image_processing/`.
            - Use `/home/sonia/ros_sonia_ws/src/proc_image_processing` as `Deployment path` value.

- Reload the CMake project using `File | Reload CMake Project`. CLion should populate all the available launch options
  up top at the left of the `Play` icon.


- Before you can launch something, you must copy once again the contents from the [clion.env.vars](clion.env.vars) file
  into the `Environment` field of the CMake Application that you want to launch. You can do that by
  using `Edit Configurations` from the toolbar at the left of the `Play` button.

**Tips:**

- The `Tools` menu is where you will find pretty much all the tools you need in order to interact with your remote
  environment. From there, you can:
    - Sync with your remote environment (this is pretty much automatic, but hey, if stuff goes sideways, you'll know
      where to look).
    - Upload or download specific files.
    - Start an SSH session.
    - Open a terminal of your remote environment directly into CLion's terminal.
    - Reload CMake Project.
    - Flush CMake cache.
    - And more!
- At the bottom of your right toolbar, you should have a tab named `Remote Host`. When you open it, you can navigate the
  filesystem of your remote environment and open files seamlessly. How cool is that?
- On your bottom toolbar, you should see a tab named `File Transfer`. This is where you can see everything related to
  the file transfers between your local and remote environment.
- No need to commit from your remote environment contrary to VSCode. Truth is, everything you modify is done locally and
  then sent to your remote environment. In other words, you won't lose your changes if you don't commit them since your
  remote environment behaves like a server where stuff is compiled and executed.

#### Running bags

- docker-compose -f docker-compose-bags.yml up octopus-telemetry
- docker-compose up clion_remote_env
- docker exec -it clion_remote_env /bin/bash
- cd /bags/
- rosbag play -l something.bag
- rosrun image_transport republish compressed in:=/camera_array/cam0/image_raw raw out:=/camera_array/cam0/image_raw

---

## Code Quality

This project uses [SonarCloud](https://sonarcloud.io/) to analyze the code base quality continuously using a GitHub
Actions Workflow.

Although, a manual analysis can also be performed and submitted on SonarCloud by following these steps:

- Copy `.env.example` a template for a `.env` file. In that file, you must specify your GitHub Token (`GITHUB_TOKEN`)
  with the necessary privileges, your SonarCloud token (`SONAR_TOKEN`) and the branch name that you are currently
  on (`BRANCH`).
- Build the SonarCloud container with: `docker-compose build proc_image_processing sonarcloud`.
- Launch the SonarCloud analysis with: `docker-compose up sonarcloud`.
- Go to [SonarCloud](https://sonarcloud.io/) and see the results for your branch.

---

### Running the tests

First of all, the environment variable `NODE_CONFIG_PATH` must be set to in order for to execute some tests. For
example, since configuration files are read from disk, setting this variable allows us to use filter chain
configurations file assets for testing purposes. As such, you can either add this variable
using `export NODE_CONFIG_PATH=/test/assets/config` or by adding `NODE_CONFIG_PATH=/test/assets/config`
before `catkin_make --use-ninja run_tests`: `NODE_CONFIG_PATH=/test/assets/config catkin_make --use-ninja run_tests`.

---

### Break down into end to end tests

Explain what these tests test and why

```
Give an example
```

---

### And coding style tests

Explain what these tests test and why

```
Give an example
```

---

## Deployment

Add additional notes about how to deploy this on a live system

---

## Built With

Add additional project dependencies

* [ROS](http://wiki.ros.org/) - ROS robotic framework

---

## Versioning

We use [SemVer](http://semver.org/) for versioning. For the versions available, see
the [tags on this repository](https://github.com/your/project/tags).


---

## License

This project is licensed under the GNU License - see the [LICENSE](LICENSE) file for details
