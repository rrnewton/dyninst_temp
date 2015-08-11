# FROM ubuntu:15.10
FROM ubuntu:14.04

RUN apt-get update && apt-get install -y --force-yes \
     cmake \
     g++ \
     libboost-dev \
     libboost-system-dev \
     libboost-thread-dev \
     libiberty-dev 

RUN apt-get install -y libelf-dev libdw1 libdw-dev
# libdwarf-dev

#     clang \
#     libclang-dev \
#     llvm \

ADD . dyninst_src/

RUN cd dyninst_src/ && mkdir build && cd build && \
    cmake .. 

# Haven't nailed this down, but I seem to have problems with make -j:
RUN cd dyninst_src/build && make 
