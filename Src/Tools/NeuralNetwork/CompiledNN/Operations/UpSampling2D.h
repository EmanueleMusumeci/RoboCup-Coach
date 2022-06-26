/**
 * @author Arne Hasselbring
 */

#pragma once

#include "../CompiledNNImplBase.h"

namespace NeuralNetwork
{
  namespace CompiledNNImpl
  {
    struct UpSampling2DCompiler : public SISOOperationCompiler
    {
      struct Parameters
      {
        std::array<unsigned int, 2> size;
        InterpolationMethod method;
      };
      const Parameters p;

      UpSampling2DCompiler(const CompilationSettings& settings, const Parameters& p) : SISOOperationCompiler(settings), p(p) {}

      inline bool canBeInplace() const override { return false; }
      void initialize() override {}
      void compile(x86::Assembler& a, ActivationFunctionHandler& afHandler, const TensorPointerXf& input, const TensorPointerXf& output) const override;

      inline std::vector<unsigned int> calcOutputDimensions(const std::vector<unsigned int>& inputDimensions) const override
      {
        ASSERT(inputDimensions.size() == 3);
        return {{inputDimensions[0] * p.size[0], inputDimensions[1] * p.size[1], inputDimensions[2]}};
      }
    };
  }
}
