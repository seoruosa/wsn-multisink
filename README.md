# WSN-multisink

## Compile
```
    mkdir build
    cmake -DCPLEX_ROOT_DIR=</path/to/ilog> -DCMAKE_BUILD_TYPE=<Debug|Release> -B build/
    cmake --build build/
```

## Running
```
    ./mtz -i <instance_path> -m <model_name> -K <number_sinks> [-r]
```

## To Do
* Clean
  * util_data.h
  * ~~funcoes.cpp~~
  * ~~funcoes.h~~
  * ~~util.h~~
  * flow_based.h
  * flow_based.cpp
* Document
  * util_data.h
  * ~~util.h~~
  * ~~funcoes.h~~
  * WSN.h
* Create tests
* ~~maybe create a util_instance.h~~
* Refactor
  * main (of flow_based)