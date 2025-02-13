// Copyright 2021-2022 Arm Limited and Contributors.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <tvm_vendor/dlpack/dlpack.h>
#include <tvm_vendor/tvm/runtime/c_runtime_api.h>
#include <tvm_vendor/tvm/runtime/module.h>
#include <tvm_vendor/tvm/runtime/packed_func.h>
#include <tvm_vendor/tvm/runtime/registry.h>

#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#ifndef TVM_UTILITY__PIPELINE_HPP_
#define TVM_UTILITY__PIPELINE_HPP_

namespace tvm_utility
{
namespace pipeline
{

class TVMArrayContainer
{
public:
  TVMArrayContainer() = default;

  TVMArrayContainer(
    std::vector<int64_t> shape, DLDataTypeCode dtype_code, int32_t dtype_bits, int32_t dtype_lanes,
    DLDeviceType device_type, int32_t device_id)
  {
    TVMArrayHandle x{};
    TVMArrayAlloc(
      &shape[0], static_cast<int>(shape.size()), dtype_code, dtype_bits, dtype_lanes, device_type,
      device_id, &x);
    handle_ = std::make_shared<TVMArrayHandle>(x);
  }

  TVMArrayHandle getArray() const { return *handle_.get(); }

private:
  std::shared_ptr<TVMArrayHandle> handle_{nullptr, [](TVMArrayHandle ptr) {
                                            if (ptr) {
                                              TVMArrayFree(ptr);
                                            }
                                          }};
};

using TVMArrayContainerVector = std::vector<TVMArrayContainer>;

/**
 * @class PipelineStage
 * @brief Base class for all types of pipeline stages.
 *
 * @tparam InputType The datatype of the input of the pipeline stage.
 * @tparam OutputType The datatype of the output from the pipeline stage.
 */
template <class InputType, class OutputType>
class PipelineStage
{
public:
  /**
   * @brief Execute the pipeline stage
   *
   * @param input The data to push into the pipeline stage. The pipeline stage
   * should not modify the input data.
   * @return The output of the pipeline
   */
  virtual OutputType schedule(const InputType & input) = 0;
  InputType input_type_indicator_;
  OutputType output_type_indicator_;
};

/**
 * @class PreProcessor
 * @brief Pre processor of the inference pipeline. In charge of converting data
 * from InputType into TVMArrayContainer format. Any necessary pre processing
 * of the data, such as image resizing or padding, should also be done in this
 * stage .
 *
 * @tparam InputType The data type of the input to the pre-processing pipeline
 * stage. Usually a ROS message type.
 */
template <class InputType>
class PreProcessor : public PipelineStage<InputType, TVMArrayContainerVector>
{
};

/**
 * @class InferenceEngine
 * @brief Pipeline stage in charge of machine learning inference.
 */
class InferenceEngine : public PipelineStage<TVMArrayContainerVector, TVMArrayContainerVector>
{
};

/**
 * @class PostProcessor
 * @brief The post processing stage of the inference pipeline. In charge of
 * converting the tensor data from the inference stage into detections in
 * OutputType, usually a ROS message format. Thing such as decoding bounding
 * boxes, non-maximum-supperssion and minimum score filtering should be done in
 * this stage.
 *
 * @tparam OutputType The data type of the output of the inference pipeline.
 * Usually a ROS message type.
 */
template <class OutputType>
class PostProcessor : public PipelineStage<TVMArrayContainerVector, OutputType>
{
};

/**
 * @class Pipeline
 * @brief Inference Pipeline. Consists of 3 stages: preprocessor, inference
 * stage and postprocessor.
 */
template <class PreProcessorType, class InferenceEngineType, class PostProcessorType>
class Pipeline
{
  using InputType = decltype(std::declval<PreProcessorType>().input_type_indicator_);
  using OutputType = decltype(std::declval<PostProcessorType>().output_type_indicator_);

public:
  /**
   * @brief Construct a new Pipeline object
   *
   * @param pre_processor a PreProcessor object
   * @param post_processor a PostProcessor object
   * @param inference_engine a InferenceEngine object
   */
  Pipeline(
    PreProcessorType pre_processor, InferenceEngineType inference_engine,
    PostProcessorType post_processor)
  : pre_processor_(pre_processor),
    inference_engine_(inference_engine),
    post_processor_(post_processor)
  {
  }

  /**
   * @brief run the pipeline. Return asynchronously in a callback.
   *
   * @param input The data to push into the pipeline
   * @return The pipeline output
   */
  OutputType schedule(const InputType & input)
  {
    auto input_tensor = pre_processor_.schedule(input);
    auto output_tensor = inference_engine_.schedule(input_tensor);
    return post_processor_.schedule(output_tensor);
  }

private:
  PreProcessorType pre_processor_{};
  InferenceEngineType inference_engine_{};
  PostProcessorType post_processor_{};
};

// Each node should be specificed with a string name and a shape
using NetworkNode = std::pair<std::string, std::vector<int64_t>>;
typedef struct
{
  // Network info
  std::string network_name;
  std::string network_backend;

  // Network files
  std::string network_module_path;
  std::string network_graph_path;
  std::string network_params_path;

  // Network data type configurations
  DLDataTypeCode tvm_dtype_code;
  int32_t tvm_dtype_bits;
  int32_t tvm_dtype_lanes;

  // Inference hardware configuration
  DLDeviceType tvm_device_type;
  int32_t tvm_device_id;

  // Network inputs
  std::vector<NetworkNode> network_inputs;

  // Network outputs
  std::vector<NetworkNode> network_outputs;
} InferenceEngineTVMConfig;

class InferenceEngineTVM : public InferenceEngine
{
public:
  explicit InferenceEngineTVM(const InferenceEngineTVMConfig & config) : config_(config)
  {
    // Get full network path
    std::string network_prefix =
      ament_index_cpp::get_package_share_directory("neural_networks_provider") + "/networks/" +
      config.network_name + "/" + config.network_backend + "/";
    std::string network_module_path = network_prefix + config.network_module_path;
    std::string network_graph_path = network_prefix + config.network_graph_path;
    std::string network_params_path = network_prefix + config.network_params_path;

    // Load compiled functions
    std::ifstream module(network_module_path);
    if (!module.good()) {
      throw std::runtime_error(
        "File " + network_module_path + " specified in inference_engine_tvm_config.hpp not found");
    }
    module.close();
    tvm::runtime::Module mod = tvm::runtime::Module::LoadFromFile(network_module_path);

    // Load json graph
    std::ifstream json_in(network_graph_path, std::ios::in);
    if (!json_in.good()) {
      throw std::runtime_error(
        "File " + network_graph_path + " specified in inference_engine_tvm_config.hpp not found");
    }
    std::string json_data(
      (std::istreambuf_iterator<char>(json_in)), std::istreambuf_iterator<char>());
    json_in.close();

    // Load parameters from binary file
    std::ifstream params_in(network_params_path, std::ios::binary);
    if (!params_in.good()) {
      throw std::runtime_error(
        "File " + network_params_path + " specified in inference_engine_tvm_config.hpp not found");
    }
    std::string params_data(
      (std::istreambuf_iterator<char>(params_in)), std::istreambuf_iterator<char>());
    params_in.close();

    // Parameters need to be in TVMByteArray format
    TVMByteArray params_arr;
    params_arr.data = params_data.c_str();
    params_arr.size = params_data.length();

    // Create tvm runtime module
    tvm::runtime::Module runtime_mod = (*tvm::runtime::Registry::Get("tvm.graph_executor.create"))(
      json_data, mod, static_cast<uint32_t>(config.tvm_device_type), config.tvm_device_id);

    // Load parameters
    auto load_params = runtime_mod.GetFunction("load_params");
    load_params(params_arr);

    // Get set_input function
    set_input = runtime_mod.GetFunction("set_input");

    // Get the function which executes the network
    execute = runtime_mod.GetFunction("run");

    // Get the function to get output data
    get_output = runtime_mod.GetFunction("get_output");

    for (auto & output_config : config.network_outputs) {
      output_.push_back(TVMArrayContainer(
        output_config.second, config.tvm_dtype_code, config.tvm_dtype_bits, config.tvm_dtype_lanes,
        kDLCPU, 0));
    }
  }

  TVMArrayContainerVector schedule(const TVMArrayContainerVector & input)
  {
    // Set input(s)
    for (uint32_t index = 0; index < input.size(); ++index) {
      if (input[index].getArray() == nullptr) {
        throw std::runtime_error("input variable is null");
      }
      set_input(config_.network_inputs[index].first.c_str(), input[index].getArray());
    }

    // Execute the inference
    execute();

    // Get output(s)
    for (uint32_t index = 0; index < output_.size(); ++index) {
      if (output_[index].getArray() == nullptr) {
        throw std::runtime_error("output variable is null");
      }
      get_output(index, output_[index].getArray());
    }
    return output_;
  }

private:
  InferenceEngineTVMConfig config_;
  TVMArrayContainerVector output_;
  tvm::runtime::PackedFunc set_input;
  tvm::runtime::PackedFunc execute;
  tvm::runtime::PackedFunc get_output;
};

}  // namespace pipeline
}  // namespace tvm_utility
#endif  // TVM_UTILITY__PIPELINE_HPP_
