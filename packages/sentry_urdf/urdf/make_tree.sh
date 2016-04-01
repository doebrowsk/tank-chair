echo "running..."



rosrun xacro xacro sentry.xacro > sentry.urdf
urdf_to_graphiz sentry.urdf

echo "dome"

