# Tools

## Benchmarks (CUDA)

Build:

- `cmake -GNinja .`
- `ninja`

Execute:

- `./proc_image_processing_benchmark1 --in /home/sonia/ros_sonia_ws/src/proc_image_processing/imgs/ --cpu-out /home/sonia/ros_sonia_ws/src/proc_image_processing/imgs/out/cpu/ --gpu-out /home/sonia/ros_sonia_ws/src/proc_image_processing/imgs/out/gpu`
- `./proc_image_processing_benchmark2 --in /home/sonia/ros_sonia_ws/src/proc_image_processing/imgs/ --cpu-out /home/sonia/ros_sonia_ws/src/proc_image_processing/imgs/out/cpu/ --gpu-out /home/sonia/ros_sonia_ws/src/proc_image_processing/imgs/out/gpu`

Args:

- `--in`: input directory of images to use for benchmark
- `--cpu-out`: output directory of images produced by cpu variant of benchmark
- `--gpu-out`: output directory of images produced by gpu variant of benchmark