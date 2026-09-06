FROM ubuntu:14.04

ENV DEBIAN_FRONTEND=noninteractive

# Trusty EOL: old-releases.ubuntu.com/ubuntu/dists/trusty снят (404). Рабочие индексы — на archive.
RUN rm -f /etc/apt/sources.list && rm -rf /etc/apt/sources.list.d && mkdir -p /etc/apt/sources.list.d && \
    printf '%s\n' \
      'deb http://archive.ubuntu.com/ubuntu/ trusty main restricted universe multiverse' \
      'deb http://archive.ubuntu.com/ubuntu/ trusty-updates main restricted universe multiverse' \
      'deb http://archive.ubuntu.com/ubuntu/ trusty-security main restricted universe multiverse' \
      > /etc/apt/sources.list

RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      ca-certificates \
      curl \
      git \
      make \
      patch \
      ninja-build \
      sudo \
      gcc \
      g++ \
      gcc-arm-linux-gnueabihf \
      g++-arm-linux-gnueabihf \
 && rm -rf /var/lib/apt/lists/*

RUN curl -fsSL https://repo.anaconda.com/miniconda/Miniconda3-py39_24.3.0-0-Linux-x86_64.sh -o /tmp/miniconda.sh \
 && bash /tmp/miniconda.sh -b -p /opt/conda \
 && rm -f /tmp/miniconda.sh \
 && /opt/conda/bin/conda clean -afy

ENV PATH="/opt/conda/bin:${PATH}"

RUN python -m pip install --upgrade pip setuptools wheel \
 && python -m pip install "conan>=2.0,<3.0"

# Prefer cmake.org (GitHub releases often time out from some networks / old curl).
RUN ( curl -fsSL --connect-timeout 30 --max-time 900 --retry 3 --retry-delay 10 \
        https://cmake.org/files/v3.22/cmake-3.22.1-linux-x86_64.sh -o /tmp/cmake.sh \
    || curl -fsSL --connect-timeout 30 --max-time 900 --retry 3 --retry-delay 10 \
        https://github.com/Kitware/CMake/releases/download/v3.22.1/cmake-3.22.1-linux-x86_64.sh -o /tmp/cmake.sh ) \
 && test -s /tmp/cmake.sh \
 && sh /tmp/cmake.sh --prefix=/usr/local --skip-license \
 && rm -f /tmp/cmake.sh

COPY docker/docker-entrypoint.sh /usr/local/bin/docker-entrypoint.sh
RUN chmod +x /usr/local/bin/docker-entrypoint.sh

WORKDIR /src

ENTRYPOINT ["/usr/local/bin/docker-entrypoint.sh"]
CMD ["/bin/bash", "-l"]
