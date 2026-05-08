import h5py
import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots

def process_model_data(file_path):
    """
    Extracts and cleans the HDF5 data for a single model, 
    returning the compute time in milliseconds.
    """
    with h5py.File(file_path, 'r') as f:
        toa_s = f['ai']['classifier']['toa_s'][:,0]
        compute_time_s = f['ai']['classifier']['compute_time_s'][:,0]

    df = pd.DataFrame({
        'toa_s': toa_s,
        'compute_time_s': compute_time_s
    })

    # 1. Remove the first sample
    df = df.iloc[1:].reset_index(drop=True)

    # 2. Find the first 0 in 'toa_s' and truncate the dataset
    zero_indices = df.index[df['toa_s'] == 0].tolist()
    if zero_indices:
        first_zero_idx = zero_indices[0]
        df = df.iloc[:first_zero_idx]

    # 3. Convert compute time from seconds to milliseconds
    return df['compute_time_s'] * 1000

def plot_separate_vertical_boxes(file_paths, model_names):
    """
    Creates a 1x3 grid of independent vertical box plots.
    """
    # Create a side-by-side subplot grid (1 row, 3 columns)
    fig = make_subplots(
        rows=1, 
        cols=len(file_paths), 
        shared_yaxes=False,   # Crucial: Unlinks the Y-axes so they scale independently
        subplot_titles=model_names,
        horizontal_spacing=0.1
    )
    
    for i, (path, name) in enumerate(zip(file_paths, model_names)):
        try:
            compute_times_ms = process_model_data(path)
            
            fig.add_trace(go.Box(
                y=compute_times_ms, # Mapped to Y-axis for vertical boxes
                name=name,
                boxpoints='outliers', 
                jitter=0.3,           
                marker=dict(size=4)
            ), row=1, col=i+1)
            
            # Label the Y-axis for each independent subplot
            fig.update_yaxes(title_text="Compute Time (ms)", row=1, col=i+1)
            
            # Hide the X-axis tick labels (the model name is already in the subplot title)
            fig.update_xaxes(showticklabels=False, row=1, col=i+1)
            
        except Exception as e:
            print(f"Error processing {name} ({path}): {e}")

    # Formatting the overall layout
    fig.update_layout(
        title='AI Model Inference Latency Comparison',
        height=600,
        template='plotly_white',
        showlegend=False
    )
    
    fig.show()

# --- Execution ---
if __name__ == "__main__":
    hdf5_files = [
        'data/project_RevalexoAiImu/type_Test/trial_0/ai.hdf5', 
        'data/project_RevalexoAiEgoImg/type_Test/trial_0/ai.hdf5', 
        'data/project_RevalexoAiEgoVid/type_Test/trial_0/ai.hdf5', 
    ]
    model_labels = ['IMU-only', 'Image + IMU', 'Video + IMU']

    plot_separate_vertical_boxes(hdf5_files, model_labels)
