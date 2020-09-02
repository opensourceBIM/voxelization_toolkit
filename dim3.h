#ifndef VEC_N_H
#define VEC_N_H

#include <cmath>
#include <array>
#include <limits>
#include <sstream>
#include <functional>

#include <boost/operators.hpp>

template <size_t N, typename T = size_t>
class vec_n : public boost::integer_arithmetic<vec_n<N, T>>, public boost::integer_arithmetic2<vec_n<N, T>, T> {
public:
	typedef vec_n<N, T> self_type;
	typedef T element_type;
	static const size_t num_elements = N;

private:
	std::array<T, N> ts_;

	template<size_t i = 0, typename... Tp>
	inline typename std::enable_if<i == sizeof...(Tp), void>::type assign_to(std::tuple<Tp...>& t) const {}

	template<size_t i = 0, typename... Tp>
	inline typename std::enable_if<i < sizeof...(Tp), void>::type assign_to(std::tuple<Tp...>& t) const {
		std::template get<i>(t) = get(i);
		assign_to<i + 1, Tp...>(t);
	}

	template<size_t i = 0>
	inline typename std::enable_if<i == N, void>::type print_(std::ostream& os, bool braces) const {
		if (braces) {
			os << ")";
		}
	}

	template<size_t i = 0>
	inline typename std::enable_if<i < N, void>::type print_(std::ostream& os, bool braces) const {
		if (i == 0) {
			if (braces) {
				os << "(";
			}
		} else {
			if (braces) {
				os << ",";
			}
			os << " ";
		}
		os << get(i);
		print_<i + 1>(os, braces);
	}

	template<size_t i = 0, typename Fn>
	inline typename std::enable_if<i == N, void>::type unary_map_(self_type& other, Fn fn) const {}

	template<size_t i = 0, typename Fn>
	inline typename std::enable_if<i < N, void>::type unary_map_(self_type& other, Fn fn) const {
		other.template get<i>() = fn(get<i>());
		unary_map_<i + 1>(other, fn);
	}

	template<size_t i = 0, typename DestT, typename Fn>
	inline typename std::enable_if<i == N, void>::type binary_map_(const self_type& other, DestT& dest, Fn fn) const {}

	template<size_t i = 0, typename DestT, typename Fn>
	inline typename std::enable_if<i < N, void>::type binary_map_(const self_type& other, DestT& dest, Fn fn) const {
		dest.template get<i>() = fn(get<i>(), other.get<i>());
		binary_map_<i + 1>(other, dest, fn);
	}

	template<size_t i = 0, typename DestT, typename Fn>
	inline typename std::enable_if<i == N, void>::type binary_map_scalar_(const T& other, DestT& dest, Fn fn) const {}

	template<size_t i = 0, typename DestT, typename Fn>
	inline typename std::enable_if<i < N, void>::type binary_map_scalar_(const T& other, DestT& dest, Fn fn) const {
		dest.template get<i>() = fn(get<i>(), other);
		binary_map_scalar_<i + 1>(other, dest, fn);
	}

	template<size_t i = 0, typename Fn>
	inline typename std::enable_if<i == N, T>::type reduce_(Fn fn, T v = T(0)) const {
		return v;
	}

	template<size_t i = 0, typename Fn>
	inline typename std::enable_if<i < N, T>::type reduce_(Fn fn, T v = T(0)) const {
		return reduce_<i + 1>(fn, fn(v, get<i>()));
	}

public:
	vec_n() {
		ts_.fill((T)0);
	}

	const std::array<T, N>& as_array() const {
		return ts_;
	}

	// Otherwise the variadic template constructor becomes our copy constructor
	vec_n(const self_type& other) {
		ts_ = other.ts_;
	}

	// Otherwise the variadic template constructor becomes our copy constructor
	vec_n& operator=(const self_type& other) {
		ts_ = other.ts_;
		return *this;
	}

	template<typename ... Ts>
	// typename std::enable_if<std::is_same<Ts, T>::value, Ts>::type
	explicit vec_n(Ts... ts) : ts_{ ts... } {
#if !defined(_MSC_VER) || _MSC_VER >= 1910
		static_assert(sizeof...(ts) == N, "Number of arguments does not match template type");
#endif
	}
	
	template <size_t i>
	T& get() {
		static_assert(i < N, "Index does not match template type");
		return ts_[i];
	}

	template <size_t i>
	const T& get() const {
		static_assert(i < N, "Index does not match template type");
		return ts_[i];
	}

	T& get(size_t i) {
		return ts_[i];
	}

	const T& get(size_t i) const {
		return ts_[i];
	}

	////

	vec_n<N, T> operator- () const {
		self_type dest;
		unary_map_(dest, std::negate<T>());
		return dest;
	}

	vec_n<N, T> operator+ () const {
		return *this;
	}

	vec_n<N, T>& operator++() {
		for (auto& x : ts_) {
			x += 1;
		}
		return *this;
	}

	vec_n<N, T>& operator--() {
		for (auto& x : ts_) {
			x -= 1;
		}
		return *this;
	}

	////

	vec_n<N, T>& operator+= (const vec_n<N, T>& rhs) {
		binary_map_(rhs, *this, std::plus<T>());
		return *this;
	}

	vec_n<N, T>& operator-= (const vec_n<N, T>& rhs) {
		binary_map_(rhs, *this, std::minus<T>());
		return *this;
	}

	vec_n<N, T>& operator*= (const vec_n<N, T>& rhs) {
		binary_map_(rhs, *this, std::multiplies<T>());
		return *this;
	}

	vec_n<N, T>& operator/= (const vec_n<N, T>& rhs) {
		binary_map_(rhs, *this, std::divides<T>());
		return *this;
	}

	vec_n<N, T>& operator%= (const vec_n<N, T>& rhs) {
		binary_map_(rhs, *this, std::modulus<T>());
		return *this;
	}

	////

	vec_n<N, bool> operator== (const vec_n<N, T>& rhs) const {
		vec_n<N, bool> r;
		binary_map_(rhs, r, std::equal_to<T>());
		return r;
	}

	vec_n<N, bool> operator!= (const vec_n<N, T>& rhs) const {
		vec_n<N, bool> r;
		binary_map_(rhs, r, std::not_equal_to<T>());
		return r;
	}

	vec_n<N, bool> operator== (const T& rhs) const {
		vec_n<N, bool> r;
		binary_map_scalar_(rhs, r, std::equal_to<T>());
		return r;
	}

	vec_n<N, bool> operator!= (const T& rhs) const {
		vec_n<N, bool> r;
		binary_map_scalar_(rhs, r, std::not_equal_to<T>());
		return r;
	}

	vec_n<N, bool> operator> (const vec_n<N, T>& rhs) const {
		vec_n<N, bool> r;
		binary_map_(rhs, r, std::greater<T>());
		return r;
	}

	vec_n<N, bool> operator< (const vec_n<N, T>& rhs) const {
		vec_n<N, bool> r;
		binary_map_(rhs, r, std::less<T>());
		return r;
	}

	vec_n<N, bool> operator>= (const vec_n<N, T>& rhs) const {
		vec_n<N, bool> r;
		binary_map_(rhs, r, std::greater_equal<T>());
		return r;
	}

	vec_n<N, bool> operator<= (const vec_n<N, T>& rhs) const {
		vec_n<N, bool> r;
		binary_map_(rhs, r, std::less_equal<T>());
		return r;
	}

	vec_n<N, bool> operator> (const T& rhs) const {
		vec_n<N, bool> r;
		binary_map_scalar_(rhs, r, std::greater<T>());
		return r;
	}

	vec_n<N, bool> operator< (const T& rhs) const {
		vec_n<N, bool> r;
		binary_map_scalar_(rhs, r, std::less<T>());
		return r;
	}

	vec_n<N, bool> operator>= (const T& rhs) const {
		vec_n<N, bool> r;
		binary_map_scalar_(rhs, r, std::greater_equal<T>());
		return r;
	}

	vec_n<N, bool> operator<= (const T& rhs) const {
		vec_n<N, bool> r;
		binary_map_scalar_(rhs, r, std::less_equal<T>());
		return r;
	}

	////

	vec_n<N, T>& operator+= (const T& rhs) {
		binary_map_scalar_(rhs, *this, std::plus<T>());
		return *this;
	}

	vec_n<N, T>& operator-= (const T& rhs) {
		binary_map_scalar_(rhs, *this, std::minus<T>());
		return *this;
	}

	vec_n<N, T>& operator*= (const T& rhs) {
		binary_map_scalar_(rhs, *this, std::multiplies<T>());
		return *this;
	}

	vec_n<N, T>& operator/= (const T& rhs) {
		binary_map_scalar_(rhs, *this, std::divides<T>());
		return *this;
	}

	vec_n<N, T>& operator%= (const T& rhs) {
		binary_map_scalar_(rhs, *this, std::modulus<T>());
		return *this;
	}

	////

	bool all() const {
		for (auto& x : ts_) {
			if (!x) {
				return false;
			}
		}
		return true;
	}

	bool any() const {
		for (auto& x : ts_) {
			if (!!x) {
				return true;
			}
		}
		return false;
	}

	T dot(const vec_n<N, T>& other) {
		T t = (T)0;
		// @todo compile time recursion
		for (size_t i = 0; i < N; ++i) {
			t += this->get(i) * other.get(i);
		}
		return t;
	}

	template <typename U = T, typename std::enable_if<std::is_signed<U>::value, int>::type = 0>
	vec_n<N, T> abs() {
		vec_n<N, T> r;
		unary_map_(r, [](auto&& v) {
			return ::abs(v);
		});
		return r;
	}

	vec_n<N, T> floor() {
		vec_n<N, T> r;
		unary_map_(r, [](auto&& v) {
			return std::floor(v);
		});
		return r;
	}

	vec_n<N, T> ceil() {
		vec_n<N, T> r;
		unary_map_(r, [](auto&& v) {
			return std::ceil(v);
		});
		return r;
	}

	vec_n<N, T> maximum(const vec_n<N, T>& other) {
		vec_n<N, T> r;
		binary_map_(other, r, [](auto&& lhs, auto&& rhs) {return std::max(lhs, rhs); });
		return r;
	}

	vec_n<N, T> minimum(const vec_n<N, T>& other) {
		vec_n<N, T> r;
		binary_map_(other, r, [](auto&& lhs, auto&& rhs) {return std::min(lhs, rhs); });
		return r;
	}

	void inplace_maximum(const vec_n<N, T>& other) {
		binary_map_(other, *this, [](auto&& lhs, auto&& rhs) {return std::max(lhs, rhs); });
	}

	void inplace_minimum(const vec_n<N, T>& other) {
		binary_map_(other, *this, [](auto&& lhs, auto&& rhs) {return std::min(lhs, rhs); });
	}

	template <typename T2>
	vec_n<N, T2> as() const {
		vec_n<N, T2> r;
		for (size_t i = 0; i < N; ++i) {
			r.get(i) = (T2)get(i);
		}
		return r;
	}

	T max_element() const {
		T v = std::numeric_limits<T>::min();
		for (size_t i = 0; i < N; ++i) {
			if (get(i) > v) {
				v = get(i);
			}
		}
		return v;
	}

	T min_element() const {
		T v = std::numeric_limits<T>::max();
		for (size_t i = 0; i < N; ++i) {
			if (get(i) < v) {
				v = get(i);
			}
		}
		return v;
	}

	template<typename ... Ts>
	void tie(Ts&... ts) const {
		static_assert(sizeof...(ts) == N, "Number of arguments does not match template type");
		auto refs = std::tie(ts...);
		assign_to(refs);
	}

	std::string format(bool braces=true) const {
		std::stringstream ss;
		print_(ss, braces);
		return ss.str();
	}

	////

	T sum() const {
		return reduce_<>(std::plus<T>());
	}

	template <typename U = T, typename std::enable_if<std::is_integral<U>::value, int>::type = 0>
	self_type ceil_div(T t) const {
		vec_n<N, T> r;
		for (size_t i = 0; i < N; ++i) {
			r.get(i) = integer_ceil_div(get(i), t);
		}
		return r;
	}

	template <typename U = T, typename std::enable_if<std::is_integral<U>::value, int>::type = 0>
	self_type floor_div(T t) const {
		vec_n<N, T> r;
		for (size_t i = 0; i < N; ++i) {
			r.get(i) = integer_floor_div(get(i), t);
		}
		return r;
	}

	template <size_t M = N, typename std::enable_if<M == 2U, int>::type = 0>
	T cross(const self_type& other) const {
		return get<0>() * other.get<1>() - get<1>() * other.get<0>();
	}

	template <size_t M = N, typename std::enable_if<M == 3U, int>::type = 0>
	self_type cross(const self_type& other) const {
		return self_type(
			get<2>() * other.get<3>() - get<3>() * other.get<2>(),
			get<3>() * other.get<1>() - get<1>() * other.get<3>(),
			get<1>() * other.get<2>() - get<2>() * other.get<1>()
		);
	}
};

#include <type_traits>

template <typename T, typename... ARGS>
constexpr auto make_vec(T const & value0, ARGS const & ... values) ->
vec_n<
	sizeof...(ARGS) + 1U, T> {
	return vec_n<sizeof...(ARGS) + 1U, T
	>{value0, static_cast<T>(values)...};
}
#endif
