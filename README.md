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
  * check TODOs
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
    * Have to create a destructor to the class? Need to use env.end()??
* Organize models
  * Others
    * WSN_arvore_rotulada_model_base.h   
    * WSN_repr_model_base.h
  * Flow-based
    * WSN_flow_model_0_1.h
    * ~~WSN_flow_model_0.h~~
    * WSN_flow_model_1.h -> check the differences with *WSN_flow_model_0_1.h*
    * **WSN_flow_model_2_1_base.h**
    * **WSN_flow_model_2_1.h** -> inherites from *2_1_base*
    * **WSN_flow_model_2_1_sbc.h** -> inherites from *2_1_base* plus balancing and breaking symmetries constraints
    * ~~WSN_flow_model_2.h~~         
    * **WSN_flow_model_3_base.h**      
    * **WSN_flow_model_3_check_instance.h** -> inherites from *3_base*
    * **WSN_flow_model_3_des.h** -> inherites from *3_base* plus constraints to be tested
    * WSN_flow_model_3idx.h
  * MCF        
    * **WSN_mcf_model_castro2023.h** -> inherites from *WSN_mcf_model_base*
    * **WSN_mcf_model_adasme2023.h** -> inherites from *WSN_mcf_model_base*
    * **WSN_mcf_model_base.h**
    * **WSN_mcf_model.h** -> inherites from *WSN_mcf_model_base*
    * **WSN_mcf_model_mcf_valid.h** -> inherites from *WSN_mcf_model_base*
    * ~~WSN_mcf_model_weight_on_node_benders.h~~
    * WSN_mcf_model_weight_on_node.h
    * WSN_mcf_weight_arc.h
  * MTZ
    * **WSN_mtz_castroandrade2023-bektas2014.h** -> inherites from *WSN_mtz_castroandrade2023-SBPO*
    * **WSN_mtz_castroandrade2023.h** -> inherites from *WSN_mtz_castroandrade2023-SBPO* 
    * ~~WSN_mtz_castroandrade2023-impSBPO-corrigido.h~~
    * ~~WSN_mtz_castroandrade2023-impSBPO.h~~
    * WSN_mtz_castroandrade2023-SBPO.h
    * **WSN_mtz_castroandrade2023-new_constr.h** -> inherites from *WSN_mtz_castroandrade2023-SBPO*
    * WSN_mtz_model_2_1.h
    * WSN_mtz_model_2.h
    * WSN_mtz_model.h
* Models to run experiments (com e sem restrições válidas ~ 2 dias 16 horas)
  * WSN_arvore_rotulada_model_base.h   
    * MTZ
    * Flow-based
  * WSN_repr_model_base.h
    * MTZ
    * Flow-based
  * WSN_flow_model_2_1.h
  * WSN_mcf_model.h
  * WSN_flow_model_3_base.h
  * WSN_mtz_castroandrade2023-SBPO.h