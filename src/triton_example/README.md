# triton_example
## Description

This package serves as an example package displaying how to write component nodes to be used in the pipeline and also to show the style conventions that are expected to be followed.

## Usage

Follow the usage of the `triton_pipeline` package with `<PIPELINE_TYPE> = example` and `<PIPELINE_INPUT> = 0`. You can then use the following command to send data to the example pipeline and see output from each node in the same terminal where the `triton_pipeline.launch.py` launch file was launched.

    ros2 topic pub /triton/example/component_one/in std_msgs/String '{data: "Hello World"}'

You can also see the final output of the pipeline using the command

    ros2 topic echo /triton/example/component_two/out

You should see the string `"Hello World from ComponentOne and ComponentTwo"` being output from the above command. Once this message is output 25 times, the second component notifies the pipeline manager that the example pipeline has been successfully run and it is unloaded from the pipeline.

## Contributors

- Logan Fillo (logan.fillo@gmail.com)