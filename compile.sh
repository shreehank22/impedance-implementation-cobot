mkdir custom/build
cd custom/build
cmake ../.. -DCMAKE_BUILD_TYPE=Release -DINSTALL_HEADERS_ONLY=true
make install -j
cmake ../.. -DCMAKE_BUILD_TYPE=Release -DINSTALL_HEADERS_ONLY=false
make install -j
cd ../..