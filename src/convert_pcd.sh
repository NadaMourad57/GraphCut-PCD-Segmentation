#!/bin/bash

# Set the root directory that contains your .ply files (and subfolders)
ROOT_DIR="/Users/nadamourad/Desktop/GraphCut-PCD-Segmentation/Dataset"

# Traverse all .ply files and convert them
find "$ROOT_DIR" -name "*.ply" | while read file; do
    base="${file%.ply}"
    echo "Converting $file -> ${base}.pcd"
    pcl_ply2pcd "$file" "${base}.pcd"
done
