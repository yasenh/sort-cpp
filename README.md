# sort-cpp
C++ version of SORT: Simple, online, and realtime tracking of multiple objects in a video sequence

```bash
$ docker pull jjanzic/docker-python3-opencv
$ docker build -t my_opencv .
$ ./docker_run.sh
```

## Testing Environment
- Ubuntu 16.04
- OpenCV 3.4.2
- Boost 1.58.0



Kuhn-Munkres (Hungarian) Algorithm in C++ is forked from:
https://github.com/saebyn/munkres-cpp


### Demo:

To run the tracker with the provided detections and visualize the results:

0. Download the [2D MOT 2015 benchmark dataset](https://motchallenge.net/data/2D_MOT_2015/#download)
0. Create a symbolic link to the dataset
    ```
    $ ln -s /path/to/MOT2015_challenge/data/2DMOT2015 mot_benchmark
    ```
0. Run the demo
    ```
    $ ./sort-cpp
    ```