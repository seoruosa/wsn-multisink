#pragma once

#include <memory>
#include "model_runner.h"
#include "models/WSN.h"

#include "models/WSN_flow_model_3idx.h"
#include "models/WSN_mtz_model.h"
#include "models/WSN_mtz_model_2.h"
#include "models/WSN_flow_model_1.h"
#include "models/WSN_flow_model_0_1.h"
#include "models/WSN_mtz_model_2_1.h"

#include "models/WSN_mtz_castroandrade2023.h"
#include "models/WSN_mtz_castroandrade2023-SBPO.h"
#include "models/WSN_mtz_castroandrade2023-bektas2014.h"
#include "models/WSN_mtz_castroandrade2023-new_constr.h"

#include "models/WSN_mcf_model.h"
#include "models/WSN_mcf_model_mcf_valid.h"
#include "models/WSN_mcf_model_base.h"
#include "models/WSN_mcf_model_adasme2023.h"
#include "models/WSN_mcf_model_castro2023.h"

#include "models/WSN_mcf_model_weight_on_node.h"

#include "models/WSN_flow_model_2_1.h"
#include "models/WSN_flow_model_2_1_base.h"
#include "models/WSN_flow_model_2_1_sbc.h"
#include "models/WSN_flow_model_3_base.h"
#include "models/WSN_flow_model_3.h"

#include "models/WSN_flow_model_3_check_instance.h"

#include "models/WSN_arvore_rotulada_model_mtz.h"
#include "models/WSN_arvore_rotulada_model_flow.h"

#include "models/WSN_mcf_weight_arc.h"
#include "models/WSN_repr_model_flow.h"
#include "models/WSN_repr_model_mtz.h"


/**
 * @brief Create a model runner and initialize with all models
 * 
 * @param instance is a problem instances
 * @return std::unique_ptr<ModelRunner<WSN>> is a pointer to the model runner
 */
std::unique_ptr<ModelRunner<WSN>> initialize_all_models(WSN_data &instance)
{
    std::unique_ptr<ModelRunner<WSN>> model_runner = std::make_unique<ModelRunner<WSN>>();

    (*model_runner).insert_model(WSN_flow_model_3idx(instance), "FlowModel3idx");

    (*model_runner).insert_model(WSN_flow_model_1(instance), "FlowModel1");
    (*model_runner).insert_model(WSN_flow_model_0_1(instance), "FlowModel0-1");
    (*model_runner).insert_model(WSN_flow_model_2_1_base(instance), "FlowModel2-1-base");
    (*model_runner).insert_model(WSN_flow_model_2_1(instance), "FlowModel2-1");
    (*model_runner).insert_model(WSN_flow_model_2_1_sbc(instance), "FlowModel2-1-sbc");

    // #################### Tree weight as flow ##################
    (*model_runner).insert_model(WSN_flow_model_3_base(instance), "FlowModel3-base");
    (*model_runner).insert_model(WSN_flow_model_3_valid_ineq(instance), "FlowModel3-valid-ineq");
    (*model_runner).insert_model(WSN_flow_model_3_testing_ineq(instance), "FlowModel3-testing-ineq");
    (*model_runner).insert_model(WSN_flow_model_3_check_instance(instance), "check-instance");

    // ####################### MTZ Models #######################
    (*model_runner).insert_model(WSN_mtz_model(instance), "MTZ");
    (*model_runner).insert_model(WSN_mtz_model_2(instance), "MTZ2");
    (*model_runner).insert_model(WSN_mtz_model_2_1(instance), "MTZ2-1");
    (*model_runner).insert_model(WSN_mtz_castro_andrade_2023(instance), "MTZ-castro2023");
    (*model_runner).insert_model(WSN_mtz_castro_andrade_2023_sbpo(instance), "MTZ-sbpo");
    (*model_runner).insert_model(WSN_mtz_castro_andrade_2023_bektas2014(instance), "MTZ-castro2023-bektas");
    (*model_runner).insert_model(WSN_mtz_castro_andrade_2023_new_constraints(instance), "MTZ-castro2023-new-constr");
    
    // ####################### MCF Models #######################
    (*model_runner).insert_model(WSN_mcf_model_base(instance), "MCFModel-base");
    (*model_runner).insert_model(WSN_mcf_model_mcf_valid(instance), "MCFModel-mcf-valid");
    (*model_runner).insert_model(WSN_mcf_model_castro2023(instance), "MCFModel-castro2023");
    (*model_runner).insert_model(WSN_mcf_model_adasme2023(instance), "MCFModel-adasme2023");
    (*model_runner).insert_model(WSN_mcf_model(instance), "MCFModel");
    
    (*model_runner).insert_model(WSN_mcf_model_weight_on_node(instance), "MCFModel-weight-node");
    (*model_runner).insert_model(WSN_mcf_weight_arc_model(instance), "MCF-weight-arc-Model");

    // ###################### Proxy Models ######################
    (*model_runner).insert_model(WSN_repr_model_flow_base(instance), "REPR-flow-base");
    (*model_runner).insert_model(WSN_repr_model_flow(instance), "REPR-flow");
    (*model_runner).insert_model(WSN_repr_model_mtz_base(instance), "REPR-mtz-base");
    (*model_runner).insert_model(WSN_repr_model_mtz(instance), "REPR-mtz");

    // ####################### MAR Models #######################
    (*model_runner).insert_model(WSN_arv_rot_model_mtz_base(instance), "MAR-mtz-base");
    (*model_runner).insert_model(WSN_arv_rot_model_mtz(instance), "MAR-mtz");
    (*model_runner).insert_model(WSN_arv_rot_model_flow_base(instance), "MAR-flow-base");
    (*model_runner).insert_model(WSN_arv_rot_model_flow(instance), "MAR-flow");
    

    return model_runner;
}