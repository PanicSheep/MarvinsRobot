#include "pch.h"
#include <algorithm>
#include <chrono>
#include "CircularBuffer.h"
#include "Resampler.h"
#include "Filters.h"
#include "Controllers.h"

using namespace std::chrono_literals;

struct Measurement
{
	double force;
	std::chrono::time_point<std::chrono::system_clock> time_point;
};

TEST(CircularBuffer, filled_init)
{
	CircularBuffer<double, 128> buffer(0.0);
	auto snapshot = buffer.get_snapshot();
	EXPECT_TRUE(std::ranges::all_of(snapshot, [](double value) { return value == 0.0; }));
}

TEST(CircularBuffer, single_value)
{
	double value = 13; // arbitrary

	CircularBuffer<double, 128> buffer;
	buffer.add(value);
	EXPECT_TRUE(buffer.get_snapshot(1).back() == value);
}

TEST(CircularBuffer, last_value_is_at_the_back)
{
	double value1 = 13; // arbitrary
	double value2 = 15; // arbitrary

	CircularBuffer<double, 128> buffer;
	buffer.add(value1);
	buffer.add(value2);

	EXPECT_TRUE(buffer.get_snapshot(1).back() == value2);
	EXPECT_TRUE(buffer.get_snapshot(2).back() == value2);
}

TEST(CircularBuffer, loops_around)
{
	double value1 = 13; // arbitrary
	double value2 = 15; // arbitrary

	CircularBuffer<double, 128> buffer;
	for (int i = 0; i < 127; i++)
		buffer.add(i);
	buffer.add(value1);
	buffer.add(value2);

	EXPECT_TRUE(buffer.get_snapshot(1).back() == value2);
	EXPECT_TRUE(buffer.get_snapshot(2)[0] == value1);
	EXPECT_TRUE(buffer.get_snapshot(2)[1] == value2);
}

TEST(Resample, standard_use_case)
{
	const auto now = std::chrono::system_clock::now();
	std::vector<Measurement> input = { 
		Measurement{ 0.0, now },
		Measurement{ 1.0, now + 4s }
	};

	auto output = Resample_LI(input, 1s, 3, [](const Measurement& m) { return m.force; }, [](const Measurement& m) { return m.time_point; });

	EXPECT_EQ(output.size(), 3);
	EXPECT_EQ(output[0], 0.5);
	EXPECT_EQ(output[1], 0.75);
	EXPECT_EQ(output[2], 1.0);
}

TEST(Filter, identity)
{
	FIR filter(std::vector{ 1.0 });
	std::vector<double> input = { 0.0, 2.0, 1.0 };

	std::vector<double> output = filter.Convolve(input);

	EXPECT_EQ(output.size(), 3);
	EXPECT_EQ(output[0], 0.0);
	EXPECT_EQ(output[1], 2.0);
	EXPECT_EQ(output[2], 1.0);
}

TEST(Filter, Boxcar)
{
	FIR filter = BoxcarFilter(1);
	std::vector<double> input = { 0.0, 2.0, 1.0 };

	std::vector<double> output = filter.Convolve(input);

	EXPECT_EQ(output.size(), 2);
	EXPECT_EQ(output[0], 1.0);
	EXPECT_EQ(output[1], 1.5);
}

TEST(PID_Controller, standard_use_case)
{
	PID_Controller<double> controller(1, 1, 1);
	controller.SetTarget(1.0);
	controller.SetMeasurement(0.0);

	EXPECT_EQ(controller.Output(), 3.0);
}