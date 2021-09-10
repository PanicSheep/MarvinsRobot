#pragma once

template <typename T>
class PID_Controller
{
	double Kp, Ki, Kd;
	T old_error{}, I{}, target{}, output{};
public:
	PID_Controller(double Kp, double Ki, double Kd) noexcept : Kp(Kp), Ki(Ki), Kd(Kd) {}

	T Target() const noexcept { return target; }
	T Output() const noexcept { return output; }

	void SetTarget(T value) noexcept { target = value; }

	void SetMeasurement(T measurement)
	{
		auto new_error = target - measurement;
		auto P = Kp * new_error;
		I += Ki * (new_error + old_error);
		auto D = Kd * (new_error - old_error);

		old_error = new_error;
		output = P + I + D;
	}

	T Process(T new_measurement)
	{
		SetMeasurement(new_measurement);
		return output;
	}
};