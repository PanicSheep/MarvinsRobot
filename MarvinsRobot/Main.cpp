#include <iostream>
#include <chrono>
#include "CircularBuffer.h"
#include "Resampler.h"
#include "Filters.h"
#include "Controllers.h"

using namespace std::chrono_literals;

struct Vec3
{
	double x, y, z;
};

struct Measurement
{
	Vec3 position, force;
	std::chrono::time_point<std::chrono::system_clock> time_point;
};

template <typename Duration>
std::vector<Vec3> ResamplePosition(const std::vector<Measurement>& input, const Duration& delta_t, std::size_t size)
{
	return Resample_LI(input, delta_t, size, [](const Measurement& m) { return m.position; }, [](const Measurement& m) { return m.time_point; });
}

template <typename Duration>
std::vector<Vec3> ResampleForce(const std::vector<Measurement>& input, const Duration& delta_t, std::size_t size)
{
	return Resample_LI(input, delta_t, size, [](const Measurement& m) { return m.force; }, [](const Measurement& m) { return m.time_point; });
}

int main()
{
	CircularBuffer<Measurement, 128> buffer;
	FIR filter = MovingAverage(16);
	PID_Controller<Measurement> force_controller(1, 1, 1);
	PID_Controller<Measurement> position_controller(1, 1, 1);

	auto input = buffer.get_snapshot();

	auto positions = ResamplePosition(input, 100us, 50);
	auto forces = ResampleForce(input, 100us, 50);

	auto smooth_positions = filter.Convolve(positions);
	auto smooth_forces = filter.Convolve(forces);

	

	return 0;
}