if [ $1 == client ]; then
    echo "building client ..."
    cd client
    idf.py build
    cd ..
    echo "done!"
    exit 0
elif [ $1 == host ]; then
    echo "building host ..."
    cd host
    idf.py build
    cd ..
    echo "done!"
    exit 0
elif [ $1 == tests ]; then
    echo "building tests ..."
    cd tests
    for d in */ ; do
        cd $d
        idf.py build
        cd ..
    done
    cd ..
    echo "done!"
    exit 0
elif [ $1 == all ]; then
    echo "building all ..."
    echo "building client ..."
    cd client
    idf.py build
    echo "building host ..."
    cd ../host
    idf.py build
    echo "building tests ..."
    cd ../tests
    for d in */ ; do
        cd $d
        idf.py build
        cd ..
    done
    cd ..
    echo "done!"
    exit 0
else
    echo "usage: ./build.sh [client|host|tests|all]"
    exit 1
fi