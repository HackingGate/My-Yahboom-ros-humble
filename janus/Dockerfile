FROM ubuntu:20.04

# Install dependencies
RUN apt-get update && \
    apt-get install -y \
    build-essential \
    libmicrohttpd-dev \
    libjansson-dev \
    libssl-dev \
    libsrtp2-dev \
    libsofia-sip-ua-dev \
    libglib2.0-dev \
    libopus-dev \
    libogg-dev \
    libcurl4-openssl-dev \
    liblua5.3-dev \
    pkg-config \
    gengetopt \
    libtool \
    automake \
    git \
    wget \
    cmake \
    python3 \
    python3-pip && \
    rm -rf /var/lib/apt/lists/*

# Install libnice
RUN cd /opt && \
    git clone https://gitlab.freedesktop.org/libnice/libnice && \
    cd libnice && \
    ./autogen.sh && \
    ./configure --prefix=/usr && \
    make && \
    make install

# Install usrsctp
RUN cd /opt && \
    git clone https://github.com/sctplab/usrsctp && \
    cd usrsctp && \
    ./bootstrap && \
    ./configure --prefix=/usr && \
    make && \
    make install

# Install WebRTC
RUN cd /opt && \
    git clone https://github.com/meetecho/janus-gateway.git && \
    cd janus-gateway && \
    sh autogen.sh && \
    ./configure --prefix=/opt/janus --disable-websockets --disable-data-channels --disable-rabbitmq --disable-mqtt --disable-docs --enable-post-processing && \
    make && \
    make install && \
    make configs

# Copy configuration files
COPY janus.conf /opt/janus/etc/janus/janus.jcfg
COPY janus.plugin.streaming.jcfg /opt/janus/etc/janus/janus.plugin.streaming.jcfg

EXPOSE 8088 8089 10000-10200/udp

CMD ["/opt/janus/bin/janus"]
