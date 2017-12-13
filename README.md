## Getting started

Clone the repository recursively:
```
git clone https://github.com/urbinn/urb --recursive
cd urb
```

There are two ways to run and/or develop this code. The recommended way is through Docker, but it is also possible to build it without Docker. 

### Docker

* Install Docker:  https://www.docker.com/
* Clone the repository: 

Instantiate the Docker image:

```
UNIX
./scripts/run-docker.sh `pwd`
```
```
WINDOWS
docker build -t urbinn -f Dockerfile .
docker run --name urbinn-dev -v $pwd -it urbinn /bin/bash
```

This builds the base image and mounts a shared volume (in our case the current directory) to enable local development. This will take a while as it builds `g2o` from scratch and installs a bunch of required dependencies (for an exact list see the `Dockerfile`).

### Without Docker

The created bindings depend on a specific version of `g2o`. You can either find this version in `bindings/libs/g2o` or [in this repository](https://github.com/urbinn/g2o). For build commands you can refer to `scripts/bootstrap.sh`. When `g2o` is installed execute the following steps:

```
./scripts/compile.sh
./scripts/test-bindings.sh 
python3 main.py
```
