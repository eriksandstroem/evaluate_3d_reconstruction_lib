# Evaluate F-score

This is an modified version of the F-score evaluation of 3D meshes provided by [**Thanks and Temples**](https://github.com/isl-org/TanksAndTemples/tree/master/python_toolbox/evaluation). 

For improved evaluation accuracy, this version does not crop or downsample the meshes and not automatic alignment is produced since the predicted and reference meshes are assumed to be aligned.

### Prerequisites
The library has been tested with the following dependencies

1. Python 3.8.5
2. Open3D 0.9.0
3. Matplotlib 3.3.3

## Installation

1. Clone the repository to your local directory: <pre><code>git clone https://github.com/tfy14esa/evaluate_3d_reconstruction_lib.git</code></pre>
2. Activate your virtual environment
3. Enter the root folder of the library i.e. "evaluate_3d_reconstruction"
4. Install the library: <pre><code>pip install .</code></pre>
 
### Usage

The main function of the library is 
<pre><code>def run_evaluation(pred_ply, path_to_pred_ply, scene, transformation=None):
    """
    Calculates the F-score from a predicted mesh to a reference mesh. Generates
    a directory and fills this numerical and mesh results.

    Args:
        pred_ply: string object to denote the name of predicted mesh (as a .ply file)
        scene: string object to denote the scene name (a corresponding ground 
                    truth .ply file with the name "scene + .ply" needs to exist)
        path_to_pred_ply: string object to denote the full path to the pred_ply file

    Returns:
        None
    """
</code></pre>
