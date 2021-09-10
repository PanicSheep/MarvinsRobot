#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <type_traits>
#include <vector>

// Resample latest values with Linear Interpolation
template <typename Input_type, typename Duration, typename ValueFkt, typename TimeFkt>
std::vector<std::invoke_result_t<ValueFkt, Input_type>> Resample_LI(const std::vector<Input_type>& input, const Duration& delta_t, std::size_t size, ValueFkt value_fkt, TimeFkt time_fkt)
{
	using std::lerp;

	if (input.empty())
		throw std::runtime_error("input is empty");

	auto first_time = time_fkt(input.front());
	auto last_time = time_fkt(input.back());
	auto provided_time_span = last_time - first_time;
	auto required_time_span = delta_t * (size - 1);
	if (required_time_span > provided_time_span)
		throw std::runtime_error("required time span exceeds provided time span");

	std::vector<std::invoke_result_t<ValueFkt, Input_type>> ret;
	ret.reserve(size);

	auto t = last_time - required_time_span;
	auto it = input.begin();
	for (std::size_t i = 0; i < size - 1; i++)
	{
		it = std::ranges::upper_bound(it, input.end(), t, {}, time_fkt);
		auto lower_time = time_fkt(*(it - 1));
		auto upper_time = time_fkt(*it);
		auto lower_time_value = value_fkt(*(it - 1));
		auto upper_time_value = value_fkt(*it);
		double fraction = static_cast<double>((t - lower_time).count()) / (upper_time - lower_time).count();
		ret.push_back(lerp(lower_time_value, upper_time_value, fraction));
		t += delta_t;
	}
	ret.push_back(value_fkt(input.back()));
	return ret;
}