
# The LTS works:
# FROM ubuntu:14.04

# But let's go one step newer for GCC 4.9:
FROM ubuntu:14.10

RUN apt-get update && apt-get install -y --force-yes \
     cmake \
     g++ \
     libboost-dev \
     libboost-system-dev \
     libboost-thread-dev \
     libiberty-dev

RUN apt-get install -y libelf-dev libdw1 libdw-dev

ADD . dyninst_src/

RUN cd dyninst_src/ && mkdir build && cd build && \
    cmake ..

# Next make, install, and remove the build intermediates:
# Haven't nailed this down, but I seem to have problems with make -j:
RUN cd dyninst_src/build && make && make install && \
    cd .. && rm -rf ./build
