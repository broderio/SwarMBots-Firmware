if [ $OSTYPE == "msys" ]; then
    echo "Unable to install packages for Windows ... "
    echo "Please install WSL2 and run this script from there."
    exit 1
elif [ $OSTYPE == "linux-gnu"* ]; then
    echo "Installing packages for Linux ..."
    sudo apt-get install git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
elif [ $OSTYPE == "darwin" ]; then
    echo "MacOS"
    brew install python3 cmake ninja dfu-util
else
    echo "OS not supported."
    exit 1
fi

mkdir -p lib/esp-idf 
cd lib
git submodule update --init
cd esp-idf
git checkout v5.1.1
git submodule update --init --recursive
chmod +x install.sh
./install.sh all
. ./export.sh
cd ../../
export IDF_PATH="$PWD/lib/esp-idf"
export PATH="$IDF_PATH/tools:$PATH"