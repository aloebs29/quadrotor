FROM --platform=$BUILDPLATFORM debian:bookworm-slim

ARG IMAGE_VERSION
ARG UID=1001
ARG GID=2000

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential=12.9 \
    curl=7.88.1-10+deb12u5 \
    git=1:2.39.2-1.1 \
    ca-certificates=20230311 \
    && rm -rf /var/lib/apt/lists/* \
    && groupadd -g $GID builders \
    && useradd -l -m -u $UID -g builders -G sudo ci \
    && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers \
    && echo "export PATH=/home/ci/.cargo/bin:$PATH" >> /etc/profile

ENV PATH=/home/ci/.cargo/bin:$PATH

# Install rust as non-root user so that cargo build can install packages
USER $UID
RUN curl --proto "=https" --tlsv1.2 --retry 3 -sSfL https://sh.rustup.rs | sh -s -- -y \
    --default-toolchain $IMAGE_VERSION \
    --component rust-src llvm-tools \
    --target thumbv7em-none-eabihf

RUN cargo install cargo-binutils cargo-make

WORKDIR /project
