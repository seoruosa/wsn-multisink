# WSN-multisink

## Compile
```
    mkdir build
    cmake -DCPLEX_ROOT_DIR=</path/to/ilog> -DCMAKE_BUILD_TYPE=<Debug|Release> -B build/
    cmake --build build/
```

## Running
```
    ./build/all_models -i <instance_path> -m <model_name> -K <number_sinks> [-r]
```

## To Do
* Clean
  * wsn_constructive_heur.h
  * unused models
* Document
  * models/*
  * wsn_constructive_heur.h
* Create tests 
  * wsn_constructive_heur.h
  * for model using a real instance
* Refactor
  * create folder utils
  * create iterator for nodes, arcs and edges
  * think in a way to run part of model as input argument
  * wsn.h