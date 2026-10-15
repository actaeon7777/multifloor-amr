# RViz TF threading workaround

`amr_rviz_plugins/SingleThreadedTF` subclasses the standard Humble TF transformer
and initializes its existing `TFWrapper` with `using_dedicated_thread=false`.
TF subscriptions, asynchronous transform request registration, and transform timers
then run on RViz's existing `SingleThreadedExecutor`. LaserScan, Path, Polygon,
RobotModel, and the normal TF connector remain compatible.

The captured production RViz backtrace showed its GUI thread blocked in
`BufferCore::addTransformableRequest` via `Buffer::waitForTransform`, while the
TF listener thread was blocked in the transform-ready callback invoked from
`BufferCore::testTransformableRequests`. This is the lock-order inversion in
https://github.com/ros2/geometry2/issues/992 (installed tf2/tf2_ros 0.25.23).

This is an RViz-scoped workaround, not a patch to system TF or Nav2 libraries.
It relies on the standard RViz single-threaded executor. Do not attach a separate
executor to the connector's buffer/node. TF receipt is paced by the GUI updates;
the navigation view uses 15 Hz. Existing timestamp checks and asynchronous waiting
remain active. Pose transformation uses the standard immediate lookup and
`tf2::doTransform`, so a synchronous timeout cannot block the receiving executor.

Build with `colcon build --symlink-install --packages-up-to amr_stack` after
sourcing `/opt/ros/jazzy/setup.bash`, then source `install/setup.bash`.
Test with `colcon test --packages-select amr_rviz_plugins --return-code-on-test-failure`.
The tests run on a separate local ROS domain and exercise delayed transforms,
callback serialization, transformed pose correctness, and missing-TF timeout.

Select the plugin under `Visualization Manager -> Transformation -> Current`.
The supplied navigation.rviz selects it automatically; keep this selection when
saving a custom navigation view. Returning to `rviz_default_plugins/TF` restores
the affected dedicated-thread path on the installed Humble library.

The navigation launch explicitly supplies this package prefix to its RViz child.
For an old terminal without the new AMENT_PREFIX_PATH entry, it checks the sibling
colcon installation next to amr_stack. Missing plugin registration/XML/library
causes an error before navigation nodes start, rather than a silent stock-TF fallback.
Standalone RViz still requires sourcing the workspace installation.
