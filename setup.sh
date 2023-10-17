mkdir -p lib/esp-idf 

if [[ "$OSTYPE" =~ ^msys ]]; then
    echo "Unable to install packages for Windows ... "
    echo "Please install WSL2 and run this script from there."
    exit 1
elif [[ "$OSTYPE" =~ ^linux-gnu ]]; then
    echo "Installing packages for Linux ..."
    echo "Installling LCM ..."
    sudo apt-get install git
    sudo apt update && sudo apt install build-essential g++ libglib2.0-dev cmake
    cd lib
    git clone https://github.com/lcm-proj/lcm.git
    cd lcm
    mkdir build
    cd build
    cmake ..
    make
    sudo make install
    sudo apt-get install python-dev && sudo apt-get install python3-dev
    cd ..
    cd lcm-python
    sudo python3 setup.py install
    cd ../../
    rm -rf lcm
    echo "Installing ESP-IDF dependencies ..."
    sudo apt-get install git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
elif [[ "$OSTYPE" =~ ^darwin ]]; then
    echo "Installing packages for MacOS ..."
    brew install python3 cmake ninja dfu-util lcm
else
    echo "OS not supported."
    exit 1
fi

cd lib
git submodule update --init

echo "Building LCM message types ..."
cd mbot_lcm_base
./scripts/install.sh
cd ../

echo "Building ESP-IDF ..."
cd esp-idf
git checkout v5.1.1
git submodule update --init --recursive
chmod +x install.sh
./install.sh all
. ./export.sh
cd ../../

export IDF_PATH="$PWD/lib/esp-idf"
echo $IDF_PATH
export PATH="$IDF_PATH/tools:$PATH"
echo $PATH
export LCM_INCLUDE_DIR="$PWD/lib/mbot_lcm_base/"
echo $LCM_INCLUDE_DIR