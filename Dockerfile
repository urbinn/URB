FROM ubuntu:17.10

WORKDIR /urbinn
ADD . /urbinn

RUN apt-get update && apt-get install -y --no-install-recommends \
	build-essential \
	cmake \
	libeigen3-dev \
	python3.6-dev \
	libsuitesparse-dev \
	python3-setuptools \
	python3-pip \
	libopencv-dev \
	python-opencv \
	&& rm -rf /var/lib/apt/lists/* \
	&& /urbinn/scripts/bootstrap.sh \
	&& ldconfig \
	&& pip3 install \
		pybind11 \
		numpy \
		pytest \
		json-tricks \
	&& pip3 install -r requirements.txt \
	&& cd bindings \
	&& python3 setup.py install