# triton_pipeline
## Description

This package is for creating a pipeline which can autonomously manage the states and actions of the AUV. The pipeline can be configured to run a defined sequence of actions, given as pipeline types, moving to the next action in a sequence when a stopping condition is reached. The nodes in the current running pipeline type can publish feedback which is able to trigger the stopping condition and ultimatley the transition to the next action.

## Usage

To define a pipeline type, follow the syntax given in `example.yaml` in the `config` folder which contains the parameters for the `pipeline_manager`. After defining your pipeline type, you have to add it to `PipelineType.msg` in `triton_interfaces` for you to be able to configure and run it. When naming a pipeline config file, you must have the name of the pipeline type match the name of the yaml (i.e after defining `example.yaml`, you have to add `string TYPE_EXAMPLE = "example"` to `PipelineType.msg`). To define a pipeline sequence, follow the syntax given in `example_sequence.yaml` in the `config` folder. The values for the sequences should all be pipeline types defined in `PipelineType.msg`. Next you can launch the pipeline as follows

    ros2 launch triton_pipeline pipeline_launch.py sequence:=<SEQUENCE_CONFIG_FILE>

Where `<SEQUENCE_CONFIG_FILE>` is the name of a yaml file in the `config` folder of `triton_pipeline` which contains the `pipeline_sequence_manager` parameters (e.g see `triton_example` for a concrete example of using the pipeline).



## Nodes

- `pipeline_manager` : A standalone node used to manage a component container to use it as a pipeline.

    ### Subscribed Topics
    - `pipeline_feedback` (`triton_interfaces/msg/PipelineFeedback.msg`) : Accepts feedback from nodes in the pipeline.
    
    ### Services
    - `configure_pipeline` (`triton_interfaces/srv/ConfigurePipeline.srv`) : Service for configuring the pipeline.
    
    ### Action Servers
    - `run_pipeline` (`triton_interfaces/action/RunPipeline.action`) : Action for running the configured pipeline.

    ### Parameters
    - `components` (`string[]`): Declares the components to be launched in the pipeline when it is run.
    - `pkg_names` (`string[]`): The packages to which the components belong. There should be a one-to-one correspondence with each component.
    - `param_files` (`string[]`): The param files to load for each component. There should be a one-to-one correspondence with each component. If there is no param file needed for a component, you must put the empty string ('') as the param file value. Note that the param file MUST exist in the config folder of the corresponding package name.
    - `namespace` (`string`): The namespace in which the components will be launched.
    - `remap_rules` (`string[]`): Declares the topic remapping rules for the pipeline components.

- `pipeline_sequence_manager`: A standalone node used to autonomously manage the sequence of pipeline types that are run in the pipeline
    ### Parameters
    - `pipeline_sequence` (`string[]`): The sequence of pipeline types to manage. The list is ordered so that the first pipeline type ran is the first in the list and the last pipeline type ran is the last in the list

## Contributors

- Logan Fillo (logan.fillo@gmail.com)
