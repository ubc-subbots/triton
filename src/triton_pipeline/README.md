# triton_pipeline
## Description

This package is for creating a pipeline which is able to dynamically load and unload
a set of nodes, defined by a pipeline type, which aim to perform an action. The pipeline can be configured based on a set of possible pipeline types, then it can be run with an input value parameter. Running the pipeline loads the nodes given in the type's configuration file into the pipeline and supplies the input value parameter if necessary. The nodes in the pipeline can then publish feedback and the pipeline manager waits until the feedback states that the nodes have completed their action then the nodes are unloaded and an ouptut value is returned.

## Usage

To define a pipeline type, follow the syntax given in `example.yaml` in the `config` folder. After defining your pipeline type, you have to add it to `PipelineType.msg` in `triton_interfaces` for you to be able to configure and run it. To launch the pipeline (along with the required pipeline manager) use the `triton_pipeline.launch.py` launch file as such

    ros2 launch triton_pipeline triton_pipeline.launch.py

To configure the pipeline use the `ConfigurePipeline.srv` service from `triton_interfaces` as such

    ros2 service call /triton/configure_pipeline triton_interfaces/srv/ConfigurePipeline '{pipeline_type: {type:  "<PIPELINE_TYPE>"}}'

Make sure that the `type` field is one of the enumerated strings in `PipelineType.msg` in `triton_interfaces`. To run the pipeline use the `RunPipeline.action` action from `triton_interfaces` as such

    ros2 action send_goal /triton/run_pipeline triton_interfaces/action/RunPipeline '{input: <PIPELINE_INPUT>}'

Make sure that the `input` field is one of the accepted types given in `RunPipeline.action` (currently can only be an int and is never used, so can be ignored for now).

## Nodes

- `pipeline_manager` : A standalone node used to manage a component container to use it as a pipeline.

    ### Subscribed Topics
    - `/triton/pipeline_feedback` (`triton_interfaces/msg/PipelineFeedback.msg`) : Accepts feedback from nodes in the pipeline.
    
    ### Services
    - `/triton/configure_pipeline` (`triton_interfaces/srv/ConfigurePipeline.srv`) : Service for configuring the pipeline.
    
    ### Action Servers
    - `/triton/run_pipeline` (`triton_interfaces/action/RunPipeline.action`) : Action for running the configured pipeline.

    ### Parameters
    - `components` (`String[]`): Declares the components to be launched in the pipeline when it is run.
    - `remap_rules` (`String[]`): Declares the topic remapping rules for the pipeline components.
    - `pkg_name` (`String`): The package to which the components belong.
    - `namespace` (`String`): The namespace in which the components will be launched.

## Contributors

- Logan Fillo (logan.fillo@gmail.com)
