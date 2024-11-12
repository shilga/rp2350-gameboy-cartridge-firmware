FROM alpine:3.17.0

RUN apk update && \
    apk upgrade && \
    apk add git \
            subversion \
            python3 \
            py3-pip \
            curl \
            build-base \
            libusb-dev \
            bsd-compat-headers \
            cmake \
            linux-headers \
            newlib-arm-none-eabi \
            gcc-arm-none-eabi \
            clang-extra-tools \
            bison \
            flex \
            boost-dev \
            texinfo \
            zlib-dev \
            libpng-dev \
            zlib-static

RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y

RUN . "$HOME/.cargo/env" && \
    rustup target add thumbv8m.main-none-eabihf

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

# GBDK installation
ARG GBDK_PATH=/usr/share/gbdk
RUN svn checkout -q -r r14228 svn://svn.code.sf.net/p/sdcc/code/trunk /home/sdcc-r14228 && \
    cd /home/sdcc-r14228/sdcc && \
    curl -Lo gbdk-sdcc-patch-file https://github.com/gbdk-2020/gbdk-2020-sdcc/releases/download/patches/gbdk-4.2-nes_banked_nonbanked_v4_combined.diff.patch && \
    patch -p0 -f < gbdk-sdcc-patch-file && \
    ./configure \
        --disable-shared --enable-gbz80-port  --enable-z80-port  --enable-mos6502-port  --enable-mos65c02-port  --disable-mcs51-port  --disable-z180-port  --disable-r2k-port  --disable-r2ka-port  --disable-r3ka-port  --disable-tlcs90-port  --disable-ez80_z80-port  --disable-z80n-port  --disable-ds390-port  --disable-ds400-port  --disable-pic14-port  --disable-pic16-port  --disable-hc08-port  --disable-s08-port  --disable-stm8-port  --disable-pdk13-port  --disable-pdk14-port  --disable-pdk15-port  --disable-ucsim  --disable-doc  --disable-device-lib && \
    make -j$(nproc) && \
    # New sdcc build no longer copies some binaries to bin
    cp -f src/sdcc bin && \
    cp -f support/sdbinutils/binutils/sdar bin && \
    cp -f support/sdbinutils/binutils/sdranlib bin && \
    cp -f support/sdbinutils/binutils/sdobjcopy bin && \
    cp -f support/sdbinutils/binutils/sdnm bin && \
    cp -f support/cpp/gcc/cc1 bin && \
    cp -f support/cpp/gcc/cpp bin/sdcpp && \
    strip bin/* || true && \
    # remove .in mapping files, etc
    rm -f bin/*.in && \
    rm -f bin/Makefile && \
    rm -f bin/README && \
    # Move cc1 to it's special hardwired path
    mkdir libexec && \
    mkdir libexec/sdcc && \
    mv bin/cc1 libexec/sdcc && \
    # Build GBDK
    git clone -b 4.2.0 https://github.com/gbdk-2020/gbdk-2020.git /home/gbdk-2020 && \
    export SDCCDIR=/home/sdcc-r14228/sdcc && \
    cd /home/gbdk-2020 && \
    make && \
    cd build && \
    cp -R gbdk $GBDK_PATH && \
    cd /home && \
    rm -rf /home/gbdk-2020 && \
    rm -rf /home/sdcc-r14228

ENV GBDK_PATH=$GBDK_PATH
