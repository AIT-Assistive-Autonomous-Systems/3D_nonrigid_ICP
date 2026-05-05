#!/usr/bin/env bash

set -eu
set -o pipefail

cd test-eagle

export PATH="../../bin:$PATH"
export LD_LIBRARY_PATH=/usr/local/vcpkg/installed/x64-linux/lib

mkdir -p results/debug

nonrigid-icp \
    --fixed pcfix.txt \
    --movable pcmov.txt \
    --transform results/pcmov.nricp \
    --voxel_size 0.50 \
    --buffer_voxels 5 \
    --matching_mode id \
    --error_metric point_to_point \
    --num_iterations 1 \
    --weights "0.02,0.02,0.02,0.02" \
    --max_euclidean_distance 10.0 \
    --sigma_mad_factor 0 \
    --max_correspondences_per_voxel 50 \
    --debug_dir "results/debug" \
    --profiling

nonrigid-icp-transform \
    --pc_in pcmov.txt \
    --pc_out results/pcmov_transformed.txt \
    --transform results/pcmov.nricp \
    --profiling