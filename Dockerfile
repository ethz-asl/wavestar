FROM ubuntu:22.04

# Install:
# * C++ build-tools
# * Open Motion Planning Library (RRTConnect and RRT*) and its dependencies
# hadolint ignore=DL3008
RUN apt-get update && \
    apt-get install -yq --no-install-recommends \
      build-essential cmake git ca-certificates \
      libompl-dev libboost-serialization-dev libboost-filesystem-dev && \
    rm -rf /var/lib/apt/lists/*

# Pull in the code
WORKDIR /rss2025_paper499_code
COPY . .

# Build it
RUN cmake -S . -B build && cmake --build build --config Release --parallel
