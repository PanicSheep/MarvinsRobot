#pragma once
#include <functional>
#include <stdexcept>
#include <numeric>
#include <vector>
#include <type_traits>
#include <utility>

// Finite Impuls Response filter
class FIR
{
	std::vector<double> feedforward; // impulse response. 
public:
	explicit FIR(std::vector<double> feedforward) : feedforward(std::move(feedforward))
	{
		if (this->feedforward.empty())
			throw std::runtime_error("feedforward can not be empty.");
	}

	template <typename Input_type, typename Proj>
	std::vector<std::invoke_result_t<Proj, Input_type>> Convolve(const std::vector<Input_type>& input, Proj projection) const
	{
		if (input.size() < feedforward.size())
			throw std::runtime_error("input is smaller than feedforward.");

		std::size_t output_size = input.size() - feedforward.size() + 1;
		std::vector<std::invoke_result_t<Proj, Input_type>> ret;
		ret.reserve(output_size);
		for (std::size_t i = 0; i < output_size; i++)
		{
			std::invoke_result_t<Proj, Input_type> inner_product{};
			for (std::size_t j = 0; j < feedforward.size(); j++)
				inner_product += projection(input[i + j]) * feedforward[j];
			ret.push_back(inner_product);
		}
		return ret;
	}
	template <typename Input_type>
	std::vector<Input_type> Convolve(const std::vector<Input_type>& input) const
	{
		return Convolve(input, [](const auto& value) { return value; });
	}
};

inline FIR MovingAverage(std::size_t length)
{
	// from https://en.wikipedia.org/wiki/Finite_impulse_response#Moving_average_example
	return FIR{ std::vector<double>(length, 1.0 / length) };
}

// Moving average FIR of length 'order + 1'.
inline FIR BoxcarFilter(std::size_t order)
{
	// from https://en.wikipedia.org/wiki/Finite_impulse_response#Moving_average_example
	return MovingAverage(order + 1);
}

//// Infite Impuls Response filter
//// for inputs without timestamps
//template <typename Kernel_type, typename Output_type>
//class IIR
//{
//	std::vector<Kernel_type> feedforward, feedback; // impulse response
//	std::vector<Output_type> output;
//public:
//	explicit IIR(std::vector<Kernel_type> feedforward, std::vector<Kernel_type> feedback) noexcept
//		: feedforward(std::move(feedforward))
//		, feedback(std::move(feedback))
//	{
//		output = std::vector<Output_type>(this->feedback.size(), Output_type{});
//	}
//	
//	template <typename Input_type, typename Proj>
//	std::vector<std::invoke_result_t<Proj&&(Input_type)>> Convolve(const std::vector<Input_type>& input, Proj&& projection = std::identity{})
//	{
//		if (input.size() < feedforward.size())
//			throw std::runtime_error("Input is smaller than feedforward.");
//
//		std::size_t output_size = input.size() - feedforward.size();
//		std::vector<U> ret;
//		ret.reserve(output_size);
//		for (std::size_t i = 0; i < output_size; i++)
//		{
//			T forward{}, back{};
//			for (std::size_t j = 0; j < feedforward.size(); j++)
//				forward += projection(input[i + j]) * feedforward[j];
//			for (std::size_t j = 0; j < feedback.size(); j++)
//				back += output[i + j] * feedback[j];
//			ret.push_back(forward - back);
//			output.erase(output.begin());
//			output.push_back(ret.back());
//			ret.push_back(
//				inner_product(input.rbegin(), input.rend(), feedforward.rbegin(), feedforward.rend(), T{})
//				- inner_product(output.rbegin(), output.rend(), feedback.rbegin(), feedback.rend(), T{}));
//			output.erase(output.begin());
//			output.push_back(ret.back());
//		}
//		return ret;
//	}
//};