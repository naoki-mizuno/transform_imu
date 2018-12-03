# transform_imu

Python module for transforming IMU messages and quaternions using TF2.

## Usage

```python
import tf2_ros
from sensor_msgs.msg import Imu
from transform_imu import *

src_imu = Imu()
src_imu.header.frame_id = 'some_frame'
src_imu.header.stamp = rospy.Time.now()
src_imu.orientation.w = 1

tgt_frame = 'some_other_frame'
tgt_imu = tf2_ros.transform(src_imu, tgt_frame)
```

## Related projects

- [`imu_transformer`](https://github.com/ros-perception/imu_pipeline/tree/indigo-devel/imu_transformer)


## License

MIT


## Author

Naoki Mizuno (mizuno.naoki@rm.is.tohoku.ac.jp)
