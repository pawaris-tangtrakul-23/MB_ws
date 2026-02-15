#!/usr/bin/env python3
"""
Helper script to find the dataset path automatically.
Searches in common locations relative to the package.
"""
import os
import sys


def find_dataset(dataset_name="FRA532_LAB1_DATASET", sequence="fibo_floor3_seq00"):
    """
    Find dataset in common locations.

    Priority:
    1. Environment variable LAB1_DATASET_PATH
    2. Package source directory (for development)
    3. Package share directory (if installed)
    4. Current workspace src/
    """

    # 1. Check environment variable
    env_path = os.environ.get('LAB1_DATASET_PATH')
    if env_path and os.path.exists(os.path.join(env_path, sequence)):
        return os.path.join(env_path, sequence)

    # 2. Try to find via ament_index (package directories)
    try:
        from ament_index_python.packages import get_package_share_directory, get_package_prefix

        # Check package share directory (if dataset was installed)
        pkg_share = get_package_share_directory('lab1')
        share_dataset = os.path.join(pkg_share, dataset_name, sequence)
        if os.path.exists(share_dataset):
            return share_dataset

        # Check source workspace (development mode)
        workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(pkg_share))))
        src_dataset = os.path.join(workspace_root, "src", "lab1", dataset_name, sequence)
        if os.path.exists(src_dataset):
            return src_dataset
    except:
        pass

    # 3. Check common relative locations
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # From scripts/ directory, dataset is ../FRA532_LAB1_DATASET
    pkg_root = os.path.dirname(script_dir)
    pkg_dataset = os.path.join(pkg_root, dataset_name, sequence)
    if os.path.exists(pkg_dataset):
        return pkg_dataset

    # 4. Check current working directory
    cwd_dataset = os.path.join(os.getcwd(), dataset_name, sequence)
    if os.path.exists(cwd_dataset):
        return cwd_dataset

    return None


if __name__ == "__main__":
    dataset_path = find_dataset()
    if dataset_path:
        print(dataset_path)
        sys.exit(0)
    else:
        print("ERROR: Dataset not found!", file=sys.stderr)
        print("Set LAB1_DATASET_PATH environment variable or ensure dataset is in workspace", file=sys.stderr)
        sys.exit(1)
