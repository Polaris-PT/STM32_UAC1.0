if [[ "$1" == "-r" ]]; then
  rm -rf build
fi
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=TRUE -GNinja -Bbuild
cmake --build build --target all
