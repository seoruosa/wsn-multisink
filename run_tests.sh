#!/bin/bash

MODELS="FlowModel3-base FlowModel3-valid-ineq MTZ-sbpo FlowModel2-1-base FlowModel2-1 MCFModel MCFModel-base REPR-mtz-base REPR-mtz REPR-flow-base REPR-flow MAR-mtz-base MAR-mtz MAR-flow-base MAR-flow"

for model in $MODELS; do
    for instance in $(ls instances); do
        for i in {1..8}; do
            echo $instance - $model - $i
            ./build/all_models -i instances/$instance -m $model -K $i -r
            # ./build/all_models -i instances/den20mtsNodes_20.txt -m $model -K $i
        done
    done
done