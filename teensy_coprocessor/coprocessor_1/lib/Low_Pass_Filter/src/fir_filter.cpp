/*
	This is a very simple C++ implementation of a floating point FIR filter.
 */

template <typename FloatType, int num_coeffs, const FloatType* coeffs>
class FirFilter {
public:
	
	FirFilter():
		current_index_(0) {
		for(int i = 0; i < num_coeffs; ++i)
		history_[i] = 0.0;
	}
	
	void put(FloatType value) {
		history_[current_index_++] = value;
		if(current_index_ == num_coeffs)
			current_index_ = 0;
	}

	FloatType get() {

		FloatType output = 0.0;

		int index = current_index_;

		for (int i = 0; i < num_coeffs; ++i) {
			if (index != 0) {
				--index;
			} else {
				index = num_coeffs - 1;
			}
			output += history_[index] * coeffs[i];
		}

		return output;
	}	

private:
	FloatType history_[num_coeffs];
	int current_index_;
};

/* This is a test program that shows how to use the FirFilter class. */

#include <iostream>
#include <assert.h>
#include <math.h>

#define FILTER1_LENGTH 4

float filter1_coeffs[FILTER1_LENGTH] = {
	0.1,
	0.2,
	0.3,
	0.4
};

bool approx_equal(float a, float b) {
	return fabs(a-b) < 1e-6;
}

int main() {
	FirFilter<float, FILTER1_LENGTH, filter1_coeffs> filter1;
	
	filter1.put(10.0);
	assert(approx_equal(filter1.get(), 1));
	filter1.put(10.0);
	assert(approx_equal(filter1.get(), 3));
	filter1.put(10.0);
	assert(approx_equal(filter1.get(), 6));
	filter1.put(10.0);
	assert(approx_equal(filter1.get(), 10));
	
	filter1.put(0.0);
	assert(approx_equal(filter1.get(), 9));
	filter1.put(0.0);
	assert(approx_equal(filter1.get(), 7));
	filter1.put(0.0);
	assert(approx_equal(filter1.get(), 4));
	filter1.put(0.0);
	assert(approx_equal(filter1.get(), 0));
	
	std::cout << "Tests OK." << std::endl;
	
	return 0;
}