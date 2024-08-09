#!/bin/bash

# MODELS="FlowModel3-base FlowModel3-valid-ineq MTZ-sbpo FlowModel2-1-base FlowModel2-1 MCFModel MCFModel-base REPR-mtz-base REPR-mtz REPR-flow-base REPR-flow MAR-mtz-base MAR-mtz MAR-flow-base MAR-flow"
# INSTANCES="instances"
# INSTANCES=~/Downloads/Instâncias/not-so-big
# INSTANCES=~/Downloads/Instâncias/big
# INSTANCES=~/code/wsn-intro/wsn-instance-gen/instances-gen-010824/instances
INSTANCES=~/code/wsn-intro/wsn-instance-gen/instances-modified
MODELS="FlowModel3-testing-ineq"
# MODELS="check-instance"

for model in $MODELS; do
    for instance in $(ls $INSTANCES); do
        for i in {1..8}; do
            echo $instance - $model - $i
            ./build/all_models -i $INSTANCES/$instance -m $model -K $i
            # ./build/all_models -i instances/den20mtsNodes_20.txt -m $model -K $i
        done
    done
done