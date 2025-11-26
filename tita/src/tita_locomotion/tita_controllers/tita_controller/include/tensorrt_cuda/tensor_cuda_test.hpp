#include <NvInfer.h>
#include <cuda_runtime_api.h>

#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

class Logger : public nvinfer1::ILogger
{
  void log(Severity severity, const char * msg) noexcept override
  {
    (void)severity;
    (void)msg;
  }
};

class CudaTest
{
private:
  bool cuda_init = false;
  nvinfer1::ICudaEngine * get_engine(const std::string & engine_file_path);
  void cuda_memory_init(void);
  // Cuda pipeline config.
  float * buffers[3];
  size_t input_size_0 = 33  * sizeof(float);
  // size_t input_size_1 = 33  * 10 * sizeof(float);
  size_t output_size = 8 * sizeof(float);
  cudaStream_t stream;
  nvinfer1::ICudaEngine * engine_;
  nvinfer1::IExecutionContext * context;
  Logger gLogger;

public:
  void do_inference(const float * input_0,float * output);
  bool get_cuda_init(void);
  explicit CudaTest(const std::string & engine_file_path);
  ~CudaTest();
};

