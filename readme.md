# # hdmapping-to-ros1 | ros1-to-hdmapping | hdmapping-to-ros2| ros2-to-hdmapping

# Simlified instruction

## Step 1 (prepare code)
```shell
mkdir -p ~/hdmapping-benchmark
cd ~/hdmapping-benchmark
git clone https://github.com/MapsHD/mandeye_to_bag.git --recursive
```

## Step 2 (build docker)
```shell
cd ~/hdmapping-benchmark/mandeye_to_bag
docker build -t mandeye-ws_noetic --target ros1 .
docker build -t mandeye-ws_humble --target ros2 .
```

## Step 3 (run docker)
```shell
cd ~/hdmapping-benchmark/mandeye_to_bag
chmod +x mandeye-convert.sh 
./mandeye-convert.sh <input_hdmapping_folder> <output_folder> hdmapping-to-ros1
./mandeye-convert.sh <input_hdmapping_folder> <output_folder> hdmapping-to-ros2
./mandeye-convert.sh <input_ros1_bag> <output_folder> ros1-to-hdmapping
./mandeye-convert.sh <input_ros2_folder> <output_folder> ros2-to-hdmapping
```

## Dependencies

```shell
sudo apt update
sudo apt install -y docker.io
sudo usermod -aG docker $USER
```
## Workspace

```shell
mkdir -p ~/hdmapping-benchmark
cd ~/hdmapping-benchmark
git clone https://github.com/MapsHD/mandeye_to_bag.git --recursive
```
## Docker build
```shell
cd ~/hdmapping-benchmark/mandeye_to_bag
docker build -t mandeye-ws_noetic --target ros1 .
docker build -t mandeye-ws_humble --target ros2 .
```

## Docker run
```shell
cd ~/hdmapping-benchmark/mandeye_to_bag
chmod +x mandeye-convert.sh 
./mandeye-convert.sh <input_hdmapping_folder> <output_folder> hdmapping-to-ros1
./mandeye-convert.sh <input_hdmapping_folder> <output_folder> hdmapping-to-ros2
./mandeye-convert.sh <input_ros1_bag> <output_folder> ros1-to-hdmapping
./mandeye-convert.sh <input_ros2_folder> <output_folder> ros2-to-hdmapping
```