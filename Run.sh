echo "Run number = " $1
make clean
make
mkdir Run$1
cp Comm Run$1
cd Run$1
echo "Running search...."
./Comm $2
