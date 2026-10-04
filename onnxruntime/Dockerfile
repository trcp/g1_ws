# base image args
ARG TARGET=onboard
ARG L4T_VERSION=36.4
ARG CUDA_VERSION=12.8

# from images
## for onboard
FROM gai313/jetson:t234.r35.3.runtime AS onboard-35.3
FROM gai313/jetson:t234.r36.4.runtime AS onboard-36.4
FROM gai313/ubuntu:22.04.amd64.cuda12.8.cudnn9.toolkit AS secondary-cuda12.8


# ======================
# onnxruntime base image
# ======================
FROM onboard-${L4T_VERSION} AS download-onboard
FROM secondary-cuda${CUDA_VERSION} AS download-secondary 
FROM download-${TARGET} AS download

# onnxruntime version
ARG ONXX_VERSION=1.22.1

# clone onnxruntime
RUN git clone --recursive https://github.com/microsoft/onnxruntime -b v${ONXX_VERSION} /tmp/onnxruntime


# ======================
# onnxruntime base image
# ======================
FROM onboard-${L4T_VERSION} AS main-onboard
FROM secondary-cuda${CUDA_VERSION} AS main-secondary
FROM main-${TARGET} AS main

# get latest cmake repositories
RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null &&\
    echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ jammy main' | sudo tee /etc/apt/sources.list.d/kitware.list >/dev/null

# install build tools
RUN apt-get update &&\
    apt-get install -y \
        cmake \
        libprotobuf-dev protobuf-compiler \
        libcurl4-openssl-dev \
        libssl-dev \
        zlib1g-dev \
        python3-dev python3-pip python3-setuptools \
        build-essential \
    &&\
    rm -rf /var/lib/apt/lists/*

# upgrade numpy
RUN pip install --upgrade 'numpy<2' 'packaging' 

# get onnxruntime repositories
COPY --from=download /tmp/onnxruntime /tmp/onnxruntime

# get build script
WORKDIR /tmp/onnxruntime
COPY ./build_onnxruntime.sh ./build_onnxruntime.sh

# execute build
CMD ["./build_onnxruntime.sh"]
