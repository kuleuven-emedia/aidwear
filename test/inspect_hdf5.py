import h5py
import numpy as np
import os


def print_hdf5_structure(file_path):
    """
    Prints the structure and content of an HDF5 file.
    """
    if not os.path.exists(file_path):
        print(f"Error: File not found at '{file_path}'")
        return

    try:
        with h5py.File(file_path, "r") as f:
            f.visititems(print_item)
    except Exception as e:
        print(f"Error opening or reading HDF5 file: {e}")


def print_item(name, obj):
    """
    Callback function for h5py's visititems to print info about each object.
    """
    indent = "  " * name.count("/")
    if isinstance(obj, h5py.Group):
        print(f"{indent}📂 Group: {os.path.basename(name)} (contains {len(obj)} members)")
    elif isinstance(obj, h5py.Dataset):
        print(f"{indent}📄 Dataset: {os.path.basename(name)}")
        print(f"{indent}  - Shape: {obj.shape}")
        print(f"{indent}  - Dtype: {obj.dtype}")

        # Print a few rows of data to give a glimpse
        num_rows_to_print = min(5, obj.shape[0] if obj.ndim > 0 else 1)
        if obj.ndim == 0:  # scalar dataset
            print(f"{indent}  - Data: {obj[()]}")
        else:
            print(f"{indent}  - Data (first {num_rows_to_print} rows):")
            print(np.array2string(obj[:num_rows_to_print], prefix=f"{indent}    "))
        print(f"{indent}{'-' * 20}")


if __name__ == "__main__":
    file_path = 'data/exo.hdf5'
    print_hdf5_structure(file_path)
