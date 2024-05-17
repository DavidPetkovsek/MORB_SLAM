parent_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )
cd "$parent_path" # change directories so working directory is where the script is

set -e # set to abort on error

# apt update for good measure
sudo apt update

sudo apt install -y build-essential cmake ninja-build libssl-dev libboost-all-dev libopencv-dev libpython-all-dev 

# eigen
git clone --depth 1 --branch 3.4.0 https://gitlab.com/libeigen/eigen.git || true
cd eigen
mkdir build_dir -p
cd build_dir
cmake ..
sudo make install

cd "$parent_path"

# Pangolin
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git --depth=1 -b v0.8 || true
cd Pangolin

./scripts/install_prerequisites.sh 

mkdir build -p
cd build

cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
sudo ldconfig > /dev/null 2> /dev/null

cd "$parent_path"

# IXWebSocket
sudo apt install zlib1g zlib1g-dev libssl-dev openssl libcurl4-openssl-dev -y

git clone --depth=1 --branch=v11.4.3 https://github.com/machinezone/IXWebSocket.git || true

cd IXWebSocket
mkdir build -p
cd build
cmake -DBUILD_SHARED_LIBS=ON -DUSE_ZLIB=1 -DUSE_TLS=1 -DUSE_WS=1 ..
make -j$(nproc)
sudo make install

echo "Finished installing all of the dependencies! Now just run build.sh! You can use -jX for choosing the number of workers with build.sh. default:-j$(nproc)"