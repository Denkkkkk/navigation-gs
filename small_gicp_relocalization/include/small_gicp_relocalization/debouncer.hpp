// fixed_deque_with_debounce.hpp
// C++17 - single-file example

#include <iostream>
#include <vector>
#include <optional>
#include <stdexcept>
#include <array>
#include <algorithm>

namespace small_gicp_relocalization
{

template<typename T>
class Debouncer {
public:
    // consecutive threshold (default disabled if threshold <= 1)
    explicit Debouncer(size_t consecutive_threshold = 3)
        : _consec_threshold(consecutive_threshold), _consec_count(0) {}

    // push a new sample and get the debounced output
    // (if changed, returns new stable, otherwise returns current stable)
    T push_sample_consecutive(const T& sample) {
        if (!_stable_value) {
            // initialize stable with first sample
            _stable_value = sample;
            _last_seen = sample;
            _consec_count = 1;
            return *_stable_value;
        }

        if (!_last_seen || sample != *_last_seen) {
            _last_seen = sample;
            _consec_count = 1;
        } else {
            ++_consec_count;
        }

        if (sample != *_stable_value &&
            _consec_threshold > 0 &&
            _consec_count >= _consec_threshold) {
            // promote candidate to stable
            _stable_value = sample;
        }

        return *_stable_value;
    }

    std::optional<T> stable_value() const { return _stable_value; }

    void reset() {
        _stable_value.reset();
        _last_seen.reset();
        _consec_count = 0;
    }

private:
    // consecutive strategy state
    size_t _consec_threshold;
    size_t _consec_count;
    std::optional<T> _stable_value;
    std::optional<T> _last_seen;
};

}