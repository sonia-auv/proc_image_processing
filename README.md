# proc_image_processing

![Docker Image CI - Master Branch](https://github.com/sonia-auv/proc_image_processing/workflows/Docker%20Image%20CI%20-%20Master%20Branch/badge.svg)
![Docker Image CI - Develop Branch](https://github.com/sonia-auv/proc_image_processing/workflows/Docker%20Image%20CI%20-%20Develop%20Branch/badge.svg?branch=develop)
![GitHub release (latest by date)](https://img.shields.io/github/v/release/sonia-auv/proc_image_processing)
![Average time to resolve an issue](https://isitmaintained.com/badge/resolution/sonia-auv/proc_image_processing.svg)

*Please read the instructions and fill in the blanks*

One Paragraph of project description goes here

## Getting Started

Clone current project by using following command :

```bash
    git clone git@github.com:sonia-auv/proc_image_processing.git
```

These instructions will get you a copy of the project up and running on your local machine for development and testing
purposes. See deployment for notes on how to deploy the project on a live system.

**IMPORTANT :** *If you have just imported your repository, please follow the instructions
in* [BOOTSTRAP.md](BOOTSTRAP.md) (Once the bootstrap completed, you can remove this comment from the README)

### Prerequisites

First and foremost to run the module you will need to
have [docker](https://www.docker.com/get-started?utm_source=google&utm_medium=cpc&utm_campaign=getstarted&utm_content=sitelink&utm_term=getstarted&utm_budget=growth&gclid=CjwKCAjw57b3BRBlEiwA1Imytuv9VRFX5Z0INBaD3JJNSUmadgQh7ZYWTw_r-yFn2S4XjZTsLbNnnBoCPsIQAvD_BwE)
installed.

To validate your installation of docker, simply type in

```
docker -v
```

If you receive an output in the likes of :

```
Docker version 19.03.5, build 633a0ea
```

It means you have it installed. If not follow instructions on how to install it for your OS.

### Installing

#### VSCode

TODO document how to run this module with VSCode

#### CLion

CLion provides more features than VSCode and is typically more powerful in several use case.

To get your environment setup with it, follow these steps:

- Build the remote environment Docker image:
    - CPU mode: `docker-compose -f docker-compose-cpu.yml build`.
    - GPU mode: `docker-compose -f docker-compose-gpu.yml build`.


- Launch the remote environment:
    - CPU mode: `docker-compose -f docker-compose-cpu.yml up clion_cpu_remote_env`.
    - GPU mode: `docker-compose -f docker-compose-cpu.yml up clion_gpu_remote_env`.


- Configure your Toolchain:
    - Go to `File | Settings | Build, Execution, Deployment | Toolchains`.
    - Add a new `Remote Host` Toolchain:
        - Give it a name, like `Remote Environment`.
        - Configure the credentials by clicking on the kog icon:
            - Give it a name, like `Remote Environment`.
            - Use `127.0.0.1` as `Host`.
            - Use `2222` as `Port`.
            - Use `sonia` as `User name`.
            - Use `test` as `Password`.
            - Leave the remaining fields with their default values.
            - Try using the `Test connection` button to make sure CLion can SSH into your remote environment.
        - Once you are done with the credentials, CLion should find CMake, Make, the C Compiler, the C++ Compiler and
          the Debugger.


- Configure CMake:
    - Go to `File | Settings | Build, Execution, Deployment | CMake`.
    - Add a new profile:
        - Use `Debug` as the `Build type`.
        - Use the toolchain name you just configured as the `Toolchain`.
        - Use `build` as the `Build directory`.
        - Copy and paste the contents from the [clion.env.vars](clion.env.vars) file as the `Environment`.
        - Make sure that `Include system environment variables` is checked when you click on the small icon at the end
          of the `Environment` field.


- Configure Deployment:
    - Go to `File | Settings | Build, Execution, Deployment | Deployment`.
    - Add a new `SFTP` deployment:
        - Give it a name, like `Remote Environment`.
        - Use your configured credentials as `SSH configuration`. Test that the connection still works!
        - Go to the `Mappings` tab:
            - Use `/home/sonia/ros_sonia_ws/src/proc_image_processing` as `Deployment path`.
            - Use the path of proc_image_processing as `Local path`. For
              example: `/home/yourusername/CLionProjects/proc_image_processing/`.

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

## Running the tests

Explain how to run the automated tests for this system

### Break down into end to end tests

Explain what these tests test and why

```
Give an example
```

### And coding style tests

Explain what these tests test and why

```
Give an example
```

## Deployment

Add additional notes about how to deploy this on a live system

## Built With

Add additional project dependencies

* [ROS](http://wiki.ros.org/) - ROS robotic framework

## Versioning

We use [SemVer](http://semver.org/) for versioning. For the versions available, see
the [tags on this repository](https://github.com/your/project/tags).

## License

This project is licensed under the GNU License - see the [LICENSE](LICENSE) file for details
