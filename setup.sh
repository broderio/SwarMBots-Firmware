mkdir -p lib/esp-idf 
cd lib
git submodule update --init
cd esp-idf
git checkout v5.1.1
git submodule update --init --recursive
cd ../../
export IDF_PATH="$PWD/lib/esp-idf"
export PATH="$IDF_PATH/tools:$PATH"