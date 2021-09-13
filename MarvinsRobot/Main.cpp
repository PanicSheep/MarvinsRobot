#include <iostream>
#include <chrono>
#include <random>
#include <mutex>
#include <thread>
#include <string>
#include "CircularBuffer.h"
#include "Resampler.h"
#include "Filters.h"
#include "Controllers.h"

using namespace std::chrono_literals;

struct Vec3
{
	double x, y, z;

	Vec3& operator+=(Vec3 o) { x += o.x; y += o.y; z += o.z; return *this; }
	Vec3& operator-=(Vec3 o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
	Vec3& operator*=(double m) { x *= m; y *= m; z *= m; return *this; }
	Vec3& operator/=(double m) { x /= m; y /= m; z /= m; return *this; }
};

Vec3 operator+(Vec3 l, Vec3 r) { return l += r; }
Vec3 operator-(Vec3 l, Vec3 r) { return l -= r; }
Vec3 operator*(Vec3 v, double m) { return v *= m; }
Vec3 operator*(double m, Vec3 v) { return v *= m; }
Vec3 operator/(Vec3 v, double m) { return v /= m; }

double norm(Vec3 v) { return std::hypot(v.x, v.y, v.z); }

std::string to_string(Vec3 v) { return std::format("({:.3f}, {:.3f}, {:.3f})", v.x, v.y, v.z); }

struct Measurement
{
	Vec3 position;
	std::chrono::time_point<std::chrono::system_clock> time_point;
};

Vec3 lerp(const Vec3& a, const Vec3& b, double t)
{
	return a * (1.0 - t) + b * t;
}

template <typename Duration>
std::vector<Vec3> Resample_LI(const std::vector<Measurement>& input, const Duration& delta_t, std::size_t size)
{
	return Resample_LI(input, delta_t, size, [](const Measurement& m) { return m.position; }, [](const Measurement& m) { return m.time_point; });
}

class Robot
{
	mutable std::mutex mtx; // protects: position, velocity, acceleration
	Vec3 position{ 0,0,0 }, velocity{ 0,0,0 }, acceleration{ 0,0,0 };
	mutable std::mt19937 gen{ std::random_device{}() };
	mutable std::uniform_real_distribution<double> observation_noise{ -1.0, 1.0 };
	std::jthread time_stepper;

	Vec3 ObservePosition() const { return position + Vec3{ observation_noise(gen), observation_noise(gen), observation_noise(gen) }; }
public:
	Robot()
	{
		time_stepper = std::jthread([&]() {
			std::mt19937 gen{ std::random_device{}() };
			std::uniform_real_distribution<double> move_noise{ 0.99, 1.01 };
			std::chrono::time_point<std::chrono::system_clock> last_update = std::chrono::system_clock::now();
			while (true)
			{
				auto now = std::chrono::system_clock::now();
				double time_step = (now - last_update).count();
				last_update = now;
				mtx.lock();
				position += velocity * time_step * move_noise(gen) * 1e-5;
				velocity += acceleration * time_step * move_noise(gen) * 1e-5;
				velocity *= 0.8;
				mtx.unlock();
				std::this_thread::sleep_for(1ms);
			}
			});
	}

	Measurement Observe() const { std::scoped_lock lock(mtx); return { ObservePosition(), std::chrono::system_clock::now() }; }
	void Accelerate(Vec3 dir) { std::scoped_lock lock(mtx); acceleration = dir; }
};

class PositionController
{
	mutable std::mutex mtx;
	Vec3 position_estimate;
	PID_Controller<Vec3> controller;

	Vec3 LimitAcceleration(Vec3 input) { return (norm(input) > 1) ? (input / norm(input)) : input; }
public:
	PositionController(double Kp, double Ki, double Kd) : controller(Kp, Ki, Kd) {}

	Vec3 PostionEstimate() const { std::scoped_lock lock(mtx); return position_estimate; }
	void SetTarget(Vec3 point) { std::scoped_lock lock(mtx); controller.SetTarget(point); }

	Vec3 Process(const std::vector<Measurement>& positions)
	{
		std::vector<Vec3> resampled_positions = Resample_LI(positions, 10ms, 50);
		std::vector<Vec3> smooth_positions = MovingAverage(32).Convolve(resampled_positions);
		std::scoped_lock lock(mtx);
		position_estimate = smooth_positions.back();
		Vec3 controller_output = controller.Process(position_estimate);
		Vec3 command = LimitAcceleration(controller_output);
		return command;
	}
};

class PathExecutor
{
	std::vector<Vec3> path;
	int index = 0;
public:
	PathExecutor()
	{
		path.push_back({ 0, 0, 0 });
		path.push_back({ 100, 200, 0 });
		path.push_back({ 800, 200, 0 });
		path.push_back({ 900, 0, 0 });
		path.push_back({ 0, 400, 0 });
	}

	void Process(PositionController& pc)
	{
		if (norm(path[index] - pc.PostionEstimate()) < 1)
		{
			std::cout << "Reached point " << index << "\n";
			index = (index + 1) % path.size();
			pc.SetTarget(path[index]);
		}
	}
};

int main()
{
	Robot robot;
	CircularBuffer<Measurement, 128> buffer;
	PositionController pc{ 5e-3, 1e-7, 7e-3 };
	PathExecutor pe;

	std::jthread observer_thread = std::jthread([&]() { while (true) { buffer.add(robot.Observe()); std::this_thread::sleep_for(10ms); } });

	// controller thread
	std::this_thread::sleep_for(1s);
	for (int i = 0; i < 1'000; i++)
	{
		pe.Process(pc);
		Vec3 command = pc.Process(buffer.get_snapshot());
		robot.Accelerate(command);
		std::cout << std::format("position estimate: {}, acceleration: {}\n", to_string(pc.PostionEstimate()), to_string(command));
		std::this_thread::sleep_for(100ms);
	}
	return 0;
}