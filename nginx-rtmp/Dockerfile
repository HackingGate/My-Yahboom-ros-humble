FROM ubuntu:20.04

RUN apt-get update && \
    apt-get install -y \
    build-essential \
    libpcre3 \
    libpcre3-dev \
    libssl-dev \
    zlib1g-dev \
    git \
    wget

RUN mkdir -p /opt/nginx && \
    cd /opt/nginx && \
    wget http://nginx.org/download/nginx-1.19.10.tar.gz && \
    tar zxvf nginx-1.19.10.tar.gz

RUN cd /opt/nginx && \
    git clone https://github.com/arut/nginx-rtmp-module.git

RUN cd /opt/nginx/nginx-1.19.10 && \
    ./configure --add-module=../nginx-rtmp-module --with-http_ssl_module && \
    make && \
    make install

COPY nginx.conf /usr/local/nginx/conf/nginx.conf

RUN mkdir -p /var/log/nginx && \
    touch /var/log/nginx/error.log && \
    touch /var/log/nginx/access.log

EXPOSE 1935 8080

CMD ["/usr/local/nginx/sbin/nginx", "-g", "daemon off;"]
