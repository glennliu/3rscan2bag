# The toolbox for 3RScan dataset ([link](https://github.com/WaldJohannaU/3RScan))
This is a simple data processing toolbox of 3RScan. It converts the 3RSCan dataset into rosbag format.

## Input data format
In ```${ROOT_DIR}/${SCAN_ID}/sequnece/```,
```
1. frame-xxxxxx.bb.txt
2. frame-xxxxxx.pose.txt 
3. frame-xxxxxx.color.jpg
4. frame-xxxxxx.depth.pgm (The raw depth, in low resolution)
5. frame-xxxxxx.rendered.depth.png (The reproduced depth, the same resolution as RGB)
6. frame-xxxxxx.rendered.instances.png
7. frame-xxxxxx.rendered.labels.png (Pixel level semantic segmentation)
```

Notice that the images that ```*.render*``` are pose-processed images by the RIO render provided by the author. Currently, we only used 1~5. 

In ```${ROOT_DIR}/${SCAN_ID}/```, global mesh and global semantic labeled instances are provided for evaluation. 

For more details, please refer to the documents of [3RScan](https://github.com/WaldJohannaU/3RScan). 

## Output RosBag format
Our toolbox creates a rosbag from the 3RScan data. The output data in the format, 

```
1. bbox: geometry_msgs/PoseArray (2D bounding box)
    poses[]
        position.x: topleft.u
        position.y: toleft.v
        position.z: global object ID (Ground-truth provided by 3RScan)
        orientation.x: bottomright.u 
        orientation.y: bottomright.v
        orientation.z: semanticTypesEnum
2. Camera_pose
3. rgb
4. raw_depth
5. depth 
6. path (for visualization)
```

A sample dataset is provided for reference [link](https://www.dropbox.com/s/nc91iinr8dlibi8/754e884c-ea24-2175-8b34-cead19d4198d.zip?dl=0).

