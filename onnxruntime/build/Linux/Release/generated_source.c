#include <onnxruntime_c_api.h>
#include <core/providers/cpu/cpu_provider_factory.h>
void* GetFunctionEntryByName(const char* name){
if(strcmp(name,"OrtGetApiBase") ==0) return (void*)&OrtGetApiBase;
if(strcmp(name,"OrtSessionOptionsAppendExecutionProvider_CPU") ==0) return (void*)&OrtSessionOptionsAppendExecutionProvider_CPU;
if(strcmp(name,"OrtSessionOptionsAppendExecutionProvider_CUDA") ==0) return (void*)&OrtSessionOptionsAppendExecutionProvider_CUDA;
return NULL;
}
