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
#include "models/WSN_mcf_model_weightAsFlow_base.h"

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

#include <limits>

/**
 * @brief Create a model runner and initialize with all models
 *
 * @param instance is a problem instances
 * @return std::unique_ptr<ModelRunner<WSN>> is a pointer to the model runner
 */
std::unique_ptr<ModelRunner<WSN>> initialize_all_models(WSN_data &instance, double upper_bound = std::numeric_limits<double>::max())
{
    std::unique_ptr<ModelRunner<WSN>> model_runner = std::make_unique<ModelRunner<WSN>>();

    (*model_runner).insert_model(WSN_flow_model_3idx(instance, upper_bound), "FlowModel3idx");

    (*model_runner).insert_model(WSN_flow_model_1(instance, upper_bound), "FlowModel1");
    (*model_runner).insert_model(WSN_flow_model_0_1(instance, upper_bound), "FlowModel0-1");
    (*model_runner).insert_model(WSN_flow_model_2_1_base(instance, upper_bound), "FlowModel2-1-base");
    (*model_runner).insert_model(WSN_flow_model_2_1(instance, upper_bound), "FlowModel2-1");
    (*model_runner).insert_model(WSN_flow_model_2_1_sbc(instance, upper_bound), "FlowModel2-1-sbc");

    // #################### Tree weight as flow ##################
    (*model_runner).insert_model(WSN_flow_model_3_base(instance, upper_bound), "FlowModel3-base");
    (*model_runner).insert_model(WSN_flow_model_3_valid_ineq(instance, upper_bound), "FlowModel3-valid-ineq");
    (*model_runner).insert_model(WSN_flow_model_3_testing_ineq(instance, upper_bound), "FlowModel3-testing-ineq");
    (*model_runner).insert_model(WSN_flow_model_3_check_instance(instance, upper_bound), "check-instance");

    // ####################### MTZ Models #######################
    (*model_runner).insert_model(WSN_mtz_model(instance, upper_bound), "MTZ");
    (*model_runner).insert_model(WSN_mtz_model_2(instance, upper_bound), "MTZ2");
    (*model_runner).insert_model(WSN_mtz_model_2_1(instance, upper_bound), "MTZ2-1");
    (*model_runner).insert_model(WSN_mtz_castro_andrade_2023(instance, upper_bound), "MTZ-castro2023");
    (*model_runner).insert_model(WSN_mtz_castro_andrade_2023_sbpo(instance, upper_bound), "MTZ-sbpo");
    (*model_runner).insert_model(WSN_mtz_castro_andrade_2023_bektas2014(instance, upper_bound), "MTZ-castro2023-bektas");
    (*model_runner).insert_model(WSN_mtz_castro_andrade_2023_new_constraints(instance, upper_bound), "MTZ-castro2023-new-constr");

    // ####################### MCF Models #######################
    (*model_runner).insert_model(WSN_mcf_model_base(instance, upper_bound), "MCFModel-base");
    (*model_runner).insert_model(WSN_mcf_model_mcf_valid(instance, upper_bound), "MCFModel-mcf-valid");
    (*model_runner).insert_model(WSN_mcf_model_castro2023(instance, upper_bound), "MCFModel-castro2023");
    (*model_runner).insert_model(WSN_mcf_model_adasme2023(instance, upper_bound), "MCFModel-adasme2023");
    (*model_runner).insert_model(WSN_mcf_model(instance, upper_bound), "MCFModel");
    (*model_runner).insert_model(WSN_mcf_model_weight_on_node(instance, upper_bound), "MCFModel-weight-node");

    // ----------------------- Alternative flows ----------------
    (*model_runner).insert_model(WSN_mcf_weight_model_base(instance, upper_bound), "MCFModel-weightAsFlow-base");
    (*model_runner).insert_model(WSN_mcf_weight_arc_model(instance, upper_bound), "MCF-weight-arc-Model");

    // ###################### Proxy Models ######################
    (*model_runner).insert_model(WSN_repr_model_flow_base(instance, upper_bound), "REPR-flow-base");
    (*model_runner).insert_model(WSN_repr_model_flow(instance, upper_bound), "REPR-flow");
    (*model_runner).insert_model(WSN_repr_model_mtz_base(instance, upper_bound), "REPR-mtz-base");
    (*model_runner).insert_model(WSN_repr_model_mtz(instance, upper_bound), "REPR-mtz");

    // ####################### MAR Models #######################
    (*model_runner).insert_model(WSN_arv_rot_model_mtz_base(instance, upper_bound), "MAR-mtz-base");
    (*model_runner).insert_model(WSN_arv_rot_model_mtz(instance, upper_bound), "MAR-mtz");
    (*model_runner).insert_model(WSN_arv_rot_model_flow_base(instance, upper_bound), "MAR-flow-base");
    (*model_runner).insert_model(WSN_arv_rot_model_flow(instance, upper_bound), "MAR-flow");

    return model_runner;
}