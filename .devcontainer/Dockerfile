FROM alpine:3.17.0

RUN apk update && \
    apk upgrade && \
    apk add git \
            curl \
            build-base \
            libusb-dev \
            bsd-compat-headers \
            cmake \
            linux-headers \
            newlib-arm-none-eabi \
            gcc-arm-none-eabi \
            clang-extra-tools

RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y

RUN . "$HOME/.cargo/env" && \
    rustup target add thumbv8m.main-none-eabihf && \
    rustup target add riscv32imac-unknown-none-elf

# Raspberry Pi Pico SDK
ARG SDK_PATH=/usr/share/pico_sdk
RUN git clone --depth 1 --branch 2.0.0 https://github.com/raspberrypi/pico-sdk $SDK_PATH && \
    cd $SDK_PATH && \
    git submodule update --init

ENV PICO_SDK_PATH=$SDK_PATH

# Picotool installation
RUN git clone --depth 1 --branch 2.0.0 https://github.com/raspberrypi/picotool.git /home/picotool && \
    cd /home/picotool && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make && \
    cp /home/picotool/build/picotool /bin/picotool && \
    rm -rf /home/picotool