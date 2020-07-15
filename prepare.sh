[ ! -d "plugins/kin_plugin/eigen" ] && ./get_eigen.sh
cd plugins/kin_plugin
./rebuild.sh
cd ../..
export GAZEBO_MODEL_PATH=$(pwd)/models/
export GAZEBO_PLUGIN_PATH=$(pwd)/plugins/

