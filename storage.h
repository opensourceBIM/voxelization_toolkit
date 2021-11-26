#ifndef VOXEL_STORAGE_H
#define VOXEL_STORAGE_H

#include "util.h"
#include "dim3.h"
#include "factory.h"

#include <set>
#include <cmath>
#include <array>
#include <tuple>
#include <vector>
#include <bitset>
#include <limits>
#include <cstring>
#include <fstream>
#include <iostream>

#define BEGIN_LOOP(i0, i1, j0, j1, k0, k1) {\
	vec_n<3, typename std::remove_reference<decltype(i0)>::type> ijk;\
	for (ijk.get(0) = i0; ijk.get(0) < i1; ++ijk.get(0)) {\
		for (ijk.get(1) = j0; ijk.get(1) < j1; ++ijk.get(1)) {\
			for (ijk.get(2) = k0; ijk.get(2) < k1; ++ijk.get(2)) {

#define BEGIN_LOOP_I(i0, i1, j0, j1, k0, k1) {\
	vec_n<3, typename std::remove_reference<decltype(i0)>::type> ijk;\
	for (ijk.get(0) = i0; ijk.get(0) <= i1; ++ijk.get(0)) {\
		for (ijk.get(1) = j0; ijk.get(1) <= j1; ++ijk.get(1)) {\
			for (ijk.get(2) = k0; ijk.get(2) <= k1; ++ijk.get(2)) {

#define BEGIN_LOOP2(v0, v1) BEGIN_LOOP(v0.template get<0>(), v1.template get<0>(), v0.template get<1>(), v1.template get<1>(), v0.template get<2>(), v1.template get<2>())

#define BEGIN_LOOP_I2(v0, v1) BEGIN_LOOP_I(v0.template get<0>(), v1.template get<0>(), v0.template get<1>(), v1.template get<1>(), v0.template get<2>(), v1.template get<2>())

#define BEGIN_LOOP_ZERO_2(v1) BEGIN_LOOP2(make_vec<decltype(v1)::element_type>((decltype(v1)::element_type)0, (decltype(v1)::element_type)0, (decltype(v1)::element_type)0), v1)

#define END_LOOP }}}}

struct voxel_desc_t {
	virtual size_t get_size_in_bits() const = 0;
};

struct bit_t : public voxel_desc_t {
	typedef bool value_type;
	typedef bool value_type_non_ref;
	typedef uint8_t storage_type;
	static const int min_value = 0;
	static const int max_value = 1;
	static const size_t size_in_bits = 1;

	virtual size_t get_size_in_bits() const { return size_in_bits; }
};

struct voxel_uint8_t : public voxel_desc_t {
	typedef const uint8_t& value_type;
	typedef uint8_t value_type_non_ref;
	typedef uint8_t storage_type;
	static const int min_value = 0;
	static const int max_value = 255;
	static const size_t size_in_bits = 8;

	virtual size_t get_size_in_bits() const { return size_in_bits; }
};

struct voxel_uint32_t : public voxel_desc_t {
	typedef const uint32_t& value_type;
	typedef uint32_t value_type_non_ref;
	typedef uint32_t storage_type;
	static const int min_value = 0;
	static const int max_value = std::numeric_limits<uint32_t>::max();
	static const size_t size_in_bits = 32;

	virtual size_t get_size_in_bits() const { return size_in_bits; }
};

enum file_part {
	file_part_meta,
	file_part_index,
	file_part_contents,
	file_part_primitives
};

enum chunk_implementations {
	CK_EMPTY, CK_EXPLICIT, CK_PLANAR, CK_CONSTANT
};

class abstract_voxel_storage {
protected:
	mutable std::array< vec_n<3, size_t>, 2 > bounds_;

public:
	abstract_voxel_storage() {
		const size_t mi = std::numeric_limits<size_t>::min();
		const size_t ma = std::numeric_limits<size_t>::max();
		bounds_[0] = make_vec(ma, ma, ma);
		bounds_[1] = make_vec(mi, mi, mi);
	}

	virtual int value_bits() const { return 1; }

	virtual ~abstract_voxel_storage() {}

	// virtual size_t size() const = 0;
	// virtual const char* const data() const = 0;

	virtual void write(file_part, std::ostream&) = 0;

	virtual void Get(const vec_n<3, size_t>& pos, void*) const = 0;
	virtual bool Get(const vec_n<3, size_t>& pos) const = 0;
	virtual void Set(const vec_n<3, size_t>& pos) = 0;
	virtual void Set(const vec_n<3, size_t>& pos, void*) = 0;

	virtual bool GetVoxelX(double& x, size_t& i) = 0;
	virtual bool GetVoxelY(double& y, size_t& j) = 0;
	virtual bool GetVoxelZ(double& z, size_t& k) = 0;

	virtual bool GetCenter(size_t i, size_t j, size_t k, double& x, double& y, double& z) = 0;

	virtual size_t GetNbX() const = 0;
	virtual size_t GetNbY() const = 0;
	virtual size_t GetNbZ() const = 0;
	vec_n<3, size_t> extents() const {
		return make_vec(GetNbX(), GetNbY(), GetNbZ());
	}

	virtual abstract_voxel_storage* inverted(void* location = nullptr) const = 0;

	virtual abstract_voxel_storage* boolean_union(const abstract_voxel_storage* other) = 0;
	virtual abstract_voxel_storage* boolean_subtraction(const abstract_voxel_storage* other) = 0;
	virtual abstract_voxel_storage* boolean_intersection(const abstract_voxel_storage* other) = 0;

	virtual void boolean_union_inplace(const abstract_voxel_storage* other) = 0;
	virtual void boolean_subtraction_inplace(const abstract_voxel_storage* other) = 0;
	virtual void boolean_intersection_inplace(const abstract_voxel_storage* other) = 0;

	virtual bool is_explicit() { return true; }
	virtual bool is_constant() { return false; }
	
	virtual abstract_voxel_storage* make_explicit(void* location = nullptr) const = 0;
	virtual abstract_voxel_storage* empty_copy() const = 0;
	virtual abstract_voxel_storage* empty_copy_as(voxel_desc_t* fmt) const = 0;
	virtual abstract_voxel_storage* copy(void* location = nullptr) const = 0;
	virtual abstract_voxel_storage* copy_as(voxel_desc_t* fmt, void* location = nullptr) const = 0;

	virtual size_t ray_intersect_n(const vec_n<3, size_t>& pos, const vec_n<3, size_t>& dir) {
		// @todo implement more efficiently for subtypes
		vec_n<3, size_t> p = pos;
		auto& e = bounds();
		size_t n = 0;
		while (true) {
			if (Get(p)) {
				n += 1;
			}
			p += dir;
			if ((p > e[1]).any()) {
				break;
			}
			if ((p < e[0]).any()) {
				break;
			}
		}
		return n;
	}

	virtual long long unsigned int count() const = 0;

	virtual const std::array< vec_n<3, size_t>, 2 >& bounds() const {
		return bounds_;
	}

	virtual const std::array< vec_n<3, size_t>, 2 >& original_bounds() const {
		return bounds();
	}

	virtual uint32_t encode() const {
		throw std::runtime_error("Cannot encode this voxel block");
	}

	virtual void release(abstract_voxel_storage* c) {
		// empty on purpose
	}
};

class regular_voxel_storage;

class set_voxel_iterator {
	regular_voxel_storage* storage_;
	vec_n<3, size_t> current_;
	std::array<vec_n<3, size_t>, 2> bounds_;
public:
	set_voxel_iterator(regular_voxel_storage* storage, vec_n<3, size_t> current);
	set_voxel_iterator& operator++();
	bool neighbour(const vec_n<3, long>& d) const;
	void* value(void*) const;
	void* neighbour_value(const vec_n<3, long>& d, void*) const;

	bool operator==(set_voxel_iterator other) const { return (current_ == other.current_).all(); }
	bool operator!=(set_voxel_iterator other) const { return (current_ != other.current_).any(); }
	const vec_n<3, size_t>& operator*() const { return current_; }
};

struct obj_export_helper {
	std::ostream* stream;
	size_t vert_counter;
	bool normals_emitted = false;
	
	obj_export_helper(std::ostream& fs, size_t n = 1)
		: stream(&fs), vert_counter(n) {}
};

class regular_voxel_storage : public abstract_voxel_storage {
protected:
	double ox_, oy_, oz_, lx_, ly_, lz_;
	double d_;
	size_t dimx_, dimy_, dimz_;

	vec_n<3, double> origin_;

public:
	set_voxel_iterator begin() {
		size_t i0, j0, k0, i1, j1, k1;
		bounds()[0].tie(i0, j0, k0);
		bounds()[1].tie(i1, j1, k1);

		BEGIN_LOOP_I(i0, i1, j0, j1, k0, k1)
			if (Get(ijk)) {
				return set_voxel_iterator(this, ijk);
			}
		END_LOOP;

		return end();
	}

	set_voxel_iterator end() {
		return set_voxel_iterator(this, bounds()[1] + vec_n<3, size_t>(1U, 1U, 1U));
	}

	regular_voxel_storage(double ox, double oy, double oz, double d, size_t dimx, size_t dimy, size_t dimz)
		: ox_(ox)
		, oy_(oy)
		, oz_(oz)
		, lx_(dimx * d)
		, ly_(dimy * d)
		, lz_(dimz * d)
		, d_(d)
		, dimx_(dimx)
		, dimy_(dimy)
		, dimz_(dimz)
		, origin_(ox_, oy_, oz_)
	{}

	bool GetVoxelX(double& x, size_t& i) {
		i = (size_t)std::floor((x - ox_) / d_);
		return true;
	}

	bool GetVoxelY(double& y, size_t& j) {
		j = (size_t)std::floor((y - oy_) / d_);
		return true;
	}

	bool GetVoxelZ(double& z, size_t& k) {
		k = (size_t)std::floor((z - oz_) / d_);
		return true;
	}

	bool GetCenter(size_t i, size_t j, size_t k, double& x, double& y, double& z) {
		x = ox_ + i * d_ + d_ / 2.;
		y = oy_ + j * d_ + d_ / 2.;
		z = oz_ + k * d_ + d_ / 2.;
		return true;
	}

	size_t GetNbX() const { return dimx_; }
	size_t GetNbY() const { return dimy_; }
	size_t GetNbZ() const { return dimz_; }

	double voxel_size() const { return d_; }

	const vec_n<3, double>& origin() const { return origin_; }

	void obj_export(std::ostream& fs, bool with_components = true, bool use_value = false);
	void obj_export(obj_export_helper& fs, bool with_components = true, bool use_value = false);
};

template<typename U>
bool RefOrBoolImpl(std::true_type, const U& u, size_t r) {
	return !!(u & (1 << r));
}

template<typename U>
const U& RefOrBoolImpl(std::false_type, const U& u, size_t r) {
	return u;
}

namespace {
	size_t size_in_bytes(size_t n, size_t in_bits) {
		if (in_bits >= 8U) {
			return n * in_bits / 8U;
		} else {
			return integer_ceil_div(n, 8U / in_bits);
		}
	}
}

template <typename T>
class continuous_voxel_storage : public regular_voxel_storage {
private:
	size_t dimz_bytes_;
	typename T::storage_type* data_;
	long long unsigned int count_;
	bool mapped_;

	void calculate_count_() {
		count_ = 0;
		if (T::size_in_bits >= 8U) {
			BEGIN_LOOP(size_t(0), dimx_, 0U, dimy_, 0U, dimz_)
				if (Get(ijk)) {
					count_ += 1;
				}
			END_LOOP;
		} else {
			for (size_t i = 0; i < size(); ++i) {
				count_ += std::bitset<8>(data_[i]).count();
			}
		} 
	}

	void calculate_bounds_() {
		const size_t mi = std::numeric_limits<size_t>::min();
		const size_t ma = std::numeric_limits<size_t>::max();
		bounds_[0] = make_vec(ma, ma, ma);
		bounds_[1] = make_vec(mi, mi, mi);

		BEGIN_LOOP(size_t(0), dimx_, 0U, dimy_, 0U, dimz_)
			if (Get(ijk)) {
				bounds_[0].inplace_minimum(ijk);
				bounds_[1].inplace_maximum(ijk);
			}
		END_LOOP;

	}
public:
	virtual int value_bits() const { return T::size_in_bits; }

	continuous_voxel_storage(double ox, double oy, double oz, double d, size_t dimx, size_t dimy, size_t dimz, void* location=nullptr)
		: regular_voxel_storage(ox, oy, oz, d, dimx, dimy, dimz)
		, dimz_bytes_(size_in_bytes(dimz, T::size_in_bits))
		, count_(0) 
	{
		if (location != nullptr) {
			data_ = new (location) typename T::storage_type[dimx_*dimy_*dimz_bytes_];
			mapped_ = true;

			// @todo: store these also in the mmap
			calculate_count_();
			calculate_bounds_();
		} else {
			data_ = new typename T::storage_type[dimx_*dimy_*dimz_bytes_]{ 0 };
			mapped_ = false;
		}
	}

	void unmap() {
		auto old = data_;
		data_ = new typename T::storage_type[dimx_*dimy_*dimz_bytes_];
		memcpy(data_, old, size());
		mapped_ = false;
	}

	~continuous_voxel_storage() {
		if (!mapped_) {
			delete[] data_;
		}
	}

	const typename T::storage_type* const data() const {
		return data_;
	}

	typename T::storage_type* data() {
		return data_;
	}

	size_t size() const {
		return dimx_ * dimy_ * dimz_bytes_;
	}

	abstract_voxel_storage* copy(void* location = nullptr) const {
		continuous_voxel_storage* c = new continuous_voxel_storage(ox_, oy_, oz_, d_, dimx_, dimy_, dimz_, location);
		memcpy(c->data_, data_, size());
		c->bounds_ = bounds_;
		c->count_ = count_;
		return c;
	}

	abstract_voxel_storage* copy_as(voxel_desc_t* fmt, void* location = nullptr) const {
		if (fmt->get_size_in_bits() == 1) {
			continuous_voxel_storage<bit_t>* c = new continuous_voxel_storage<bit_t>(ox_, oy_, oz_, d_, dimx_, dimy_, dimz_, location);
			BEGIN_LOOP(size_t(0), dimx_, 0U, dimy_, 0U, dimz_)
				if (Get(ijk)) {
					c->Set(ijk);
				}
			END_LOOP;
			return c;
		} else {
			throw std::runtime_error("Not implemented");
		}
	}

	abstract_voxel_storage* empty_copy() const {
		// @todo is this safe?
		return nullptr;
	}

	abstract_voxel_storage* empty_copy_as(voxel_desc_t* fmt) const {
		// @todo is this safe?
		return nullptr;
	}

	abstract_voxel_storage* make_explicit(void* location = nullptr) const {
		// @todo other code depends on this returning nullptr, but perhaps
		// that code should call is_explicit() instead?
		return nullptr;
	}

	void write(file_part p, std::ostream& os) {
		if (p == file_part_meta) {
			os << "CONT" << std::endl
				<< GetNbX() << std::endl
				<< GetNbY() << std::endl
				<< GetNbZ() << std::endl;
		} else if (p == file_part_contents) {
			os.write((char*) data(), size());
		}
	}

	void Set(const vec_n<3, size_t>& xyz) {
		Set(xyz, 1);
	}

	void Set(const vec_n<3, size_t>& pos, void* ptr) {
		Set(pos, *(typename T::value_type_non_ref*) ptr);
	}

	void Set(const vec_n<3, size_t>& xyz, const typename T::value_type_non_ref& v) {
		const size_t& x = xyz.get<0>();
		const size_t& y = xyz.get<1>();
		const size_t& z = xyz.get<2>();

		bool updated = false;
		if (T::size_in_bits >= 8U) {
			auto& is_set = data_[x + y * dimx_ + z * dimx_ * dimy_];
			if (!is_set) {
				count_ += 1;
			}
			if (is_set != v) {
				is_set = v;
				updated = true;
			}
		} else {
			size_t z8 = z / (8U / T::size_in_bits);
			uint8_t r = z % (8U / T::size_in_bits);
			auto& addr = data_[x + y * dimx_ + z8 * dimx_ * dimy_];
			if (!(addr & (1 << r))) {
				count_ += 1;
				addr |= 1 << r;
				updated = true;
			}				
		}
		if (updated) {
			bounds_[0].inplace_minimum(make_vec(x, y, z));
			bounds_[1].inplace_maximum(make_vec(x, y, z));
		}
	}

	typename T::value_type ValueAt(const vec_n<3, size_t>& xyz) const {
		const size_t& x = xyz.get<0>();
		const size_t& y = xyz.get<1>();
		const size_t& z = xyz.get<2>();

		if (T::size_in_bits >= 8U) {
			return data_[x + y * dimx_ + z * dimx_ * dimy_];
		} else if (T::size_in_bits < 8U)  {
			size_t z8 = z / (8U / T::size_in_bits);
			uint8_t r = z % (8U / T::size_in_bits);
			// @todo why this integral_constant, isn't size_in_bits always < 8U
			return RefOrBoolImpl(std::integral_constant<bool, T::size_in_bits < 8U>{}, data_[x + y * dimx_ + z8 * dimx_ * dimy_], r);
		}
	}

	bool Get(const vec_n<3, size_t>& xyz) const {
		return ValueAt(xyz) != 0;
	}

	void Get(const vec_n<3, size_t>& pos, void* loc) const {
		*((typename T::value_type_non_ref*) loc) = ValueAt(pos);
	}

	abstract_voxel_storage* inverted(void* location = nullptr) const {
		continuous_voxel_storage* c = new continuous_voxel_storage(ox_, oy_, oz_, d_, dimx_, dimy_, dimz_, location);
		for (size_t i = 0; i < size(); ++i) {
			if (T::size_in_bits < 8U) {
				c->data_[i] = ~data_[i];
			} else {
				c->data_[i] = data_[i] ? 0 : 1;
			}
		}
		c->count_ = (dimx_ * dimy_ * dimz_) - count();
		c->calculate_bounds_();
		return c;
	}

	abstract_voxel_storage* boolean_union(const abstract_voxel_storage* other) {
		boolean_union_inplace(other);
		return nullptr;
	}

	void boolean_union_inplace(const abstract_voxel_storage* other_) {
		const continuous_voxel_storage<T>* other = (const continuous_voxel_storage<T>*) other_;
		for (size_t i = 0; i < size(); ++i) {
			if (T::size_in_bits < 8U) {
				data_[i] |= other->data_[i];
			} else if (other->data_[i]) {
				data_[i] = other->data_[i];
			}
		}
		calculate_count_();

		const auto& other_bounds = other_->bounds();
		bounds_[0].inplace_minimum(other_bounds[0]);
		bounds_[1].inplace_maximum(other_bounds[1]);
	}

	abstract_voxel_storage* boolean_subtraction(const abstract_voxel_storage* other) {
		boolean_subtraction_inplace(other);
		return nullptr;
	}

	void boolean_subtraction_inplace(const abstract_voxel_storage* other_) {
		const continuous_voxel_storage<T>* other = (const continuous_voxel_storage<T>*) other_;
		if (this->value_bits() == other_->value_bits()) {
			for (size_t i = 0; i < size(); ++i) {
				if (T::size_in_bits < 8U) {
					data_[i] &= ~other->data_[i];
				} else if (other->data_[i]) {
					data_[i] = 0;
				}
			}
		} else if (this->value_bits() == 32 && other_->value_bits() == 1) {
			uint32_t zero = 0;
			BEGIN_LOOP(size_t(0), dimx_, 0U, dimy_, 0U, dimz_)
				if (other_->Get(ijk)) {
					Set(ijk, &zero);
				}
			END_LOOP;
		} else {
			throw std::runtime_error("Not implemented");
		}
		calculate_count_();
		calculate_bounds_();
	}

	abstract_voxel_storage* boolean_intersection(const abstract_voxel_storage* other) {
		boolean_intersection_inplace(other);
		return nullptr;
	}

	void boolean_intersection_inplace(const abstract_voxel_storage* other_) {
		if (this->value_bits() == other_->value_bits()) {
			const continuous_voxel_storage<T>* other = (const continuous_voxel_storage<T>*) other_;
			for (size_t i = 0; i < size(); ++i) {
				data_[i] &= other->data_[i];
			}
		} else if (this->value_bits() == 32 && other_->value_bits() == 1) {
			uint32_t zero = 0;
			BEGIN_LOOP(size_t(0), dimx_, 0U, dimy_, 0U, dimz_)
				if (!other_->Get(ijk)) {
					Set(ijk, &zero);
				}
			END_LOOP;
		} else {
			throw std::runtime_error("Not implemented");
		}
		calculate_count_();
		calculate_bounds_();
	}

	long long unsigned int count() const {
		return count_;
	}

	bool is_mapped() const {
		return mapped_;
	}

	/*
	size_t ray_intersect_n(const vec_n<3, size_t>& pos, const vec_n<3, size_t>& dir) {
		if ((dir != {0U, 0U, 1U}).all()) {
			throw std::runtime_error("not implemented");
		}
		if (T::size_in_bits != 1) {
			throw std::runtime_error("not implemented");
		}
		size_t z8 = pos.get<2>() / 8;
	}
	*/
};

template <typename T>
class planar_voxel_storage : public regular_voxel_storage {
private:
	size_t axis_;
	// @todo should become a map?
	std::set<size_t> offsets_;
	std::string text_;

	void build_(size_t offset) {
		if (!text_.empty()) {
			text_.push_back(',');
		}
		text_.push_back("XYZ"[axis_]);
		text_.push_back('=');
		char buff[10];
		sprintf(buff, "%d", (int)offset);
		text_ += buff;

		bounds_[0] = vec_n<3, size_t>(0U, 0U, 0U);
		bounds_[1] = extents() - make_vec<size_t>(1U, 1U, 1U);

		bounds_[0].get(axis_) = *offsets_.begin();
		bounds_[1].get(axis_) = *offsets_.rbegin();
	}

public:
	virtual int value_bits() const { return T::size_in_bits; }

	planar_voxel_storage(double ox, double oy, double oz, double d, size_t dimx, size_t dimy, size_t dimz, size_t axis, size_t offset)
		: regular_voxel_storage(ox, oy, oz, d, dimx, dimy, dimz)
		, axis_(axis)
	{
		add(offset);
	}

	const std::set<size_t>& offsets() const {
		return offsets_;
	}

	abstract_voxel_storage* copy(void* location = nullptr) const {
		planar_voxel_storage* c = new planar_voxel_storage(ox_, oy_, oz_, d_, dimx_, dimy_, dimz_, axis_, *offsets_.begin());
		for (auto it = offsets_.begin(); it != offsets_.end(); ++it) {
			c->add(*it);
		}
		return c;
	}

	abstract_voxel_storage* copy_as(voxel_desc_t* fmt, void* location = nullptr) const {
		if (fmt->get_size_in_bits() == 1) {
			planar_voxel_storage* c = new planar_voxel_storage<bit_t>(ox_, oy_, oz_, d_, dimx_, dimy_, dimz_, axis_, *offsets_.begin());
			for (auto it = offsets_.begin(); it != offsets_.end(); ++it) {
				c->add(*it);
			}
			return c;
		}
		else {
			throw std::runtime_error("Not implemented");
		}
	}

	abstract_voxel_storage* empty_copy() const {
		// @todo is this safe?
		return nullptr;
	}

	abstract_voxel_storage* empty_copy_as(voxel_desc_t* fmt) const {
		// @todo is this safe?
		return nullptr;
	}

	abstract_voxel_storage* inverted(void* location = nullptr) const {
		throw std::runtime_error("Not implemented");
	}

	size_t axis() const {
		return axis_;
	}

	void add(size_t offset) {
		auto inserted = offsets_.insert(offset);
		if (inserted.second) {
			build_(offset);
		}
	}

	const char* const data() const {
		return text_.c_str();
	}

	size_t size() const {
		return text_.size();
	}

	void write(file_part p, std::ostream& os) {
		if (p == file_part_primitives) {
			os.write(data(), size());
		}
	}

	void Set(const vec_n<3, size_t>&) {
		throw std::runtime_error("Invalid");
	}

	void Set(const vec_n<3, size_t>&, void*) {
		throw std::runtime_error("Invalid");
	}

	bool Get(const vec_n<3, size_t>& xyz) const {
		const size_t& o = xyz.get(axis_);
		return offsets_.find(o) != offsets_.end();
	}

	void Get(const vec_n<3, size_t>& xyz, void* loc) const {
		*((typename T::value_type_non_ref*)loc) = Get(xyz);
	}

	abstract_voxel_storage* boolean_union(const abstract_voxel_storage* other_) {
		throw std::runtime_error("Invalid");
	}

	abstract_voxel_storage* boolean_subtraction(const abstract_voxel_storage* other_) {
		throw std::runtime_error("Invalid");
	}

	abstract_voxel_storage* boolean_intersection(const abstract_voxel_storage* other_) {
		throw std::runtime_error("Invalid");
	}

	void boolean_union_inplace(const abstract_voxel_storage* other_) {
		throw std::runtime_error("Invalid");
	}

	void boolean_subtraction_inplace(const abstract_voxel_storage* other_) {
		throw std::runtime_error("Invalid");
	}

	void boolean_intersection_inplace(const abstract_voxel_storage* other_) {
		throw std::runtime_error("Invalid");
	}

	bool is_explicit() { return false; }

	continuous_voxel_storage<T>* make_explicit(void* location=nullptr) const {
		continuous_voxel_storage<T>* c = new continuous_voxel_storage<T>(ox_, oy_, oz_, d_, dimx_, dimy_, dimz_, location);

		std::vector<size_t> dims = { dimx_, dimy_, dimz_ };
		dims.erase(dims.begin() + axis_);

		std::vector<size_t> axes = { 0, 1, 2 };
		axes.erase(axes.begin() + axis_);

		vec_n<3, size_t> xyz;

		for (auto& offset : offsets_) {

			xyz.get(axis_) = offset;

			for (size_t i = 0; i < dims[0]; ++i) {
				xyz.get(axes[0]) = i;
				for (size_t j = 0; j < dims[1]; ++j) {
					xyz.get(axes[1]) = j;
					c->Set(xyz);
				}
			}

		}

		return c;
	}

	long long unsigned int count() const {
		std::vector<size_t> ex{ dimx_, dimy_, dimz_ };
		ex.erase(ex.begin() + axis_);
		return offsets_.size() * ex[0] * ex[1];
	}

	uint32_t encode() const {
		if (offsets_.size() > 3) {
			// @todo this is a bit silly, but probably wouldn't run into this. Except when inverting a plane!
			throw std::runtime_error("Too many plane definitions for storage in mmap");
		}
		std::array<size_t, 3> temp = { dimx_, dimy_, dimz_ };
		if (temp[axis_] >= (1 << 8)) {
			throw std::runtime_error("Planar storage dimensions too large for mmap");
		}
		uint32_t v = CK_PLANAR;
		int n = 1;
		for (auto& o : offsets_) {
			v |= o << (8 * n);
			n++;
		}
		return v;
	}
};

template <typename T>
class constant_voxel_storage : public regular_voxel_storage {
protected:
	size_t value_;
public:
	virtual int value_bits() const { return T::size_in_bits; }

	constant_voxel_storage(double ox, double oy, double oz, double d, size_t dimx, size_t dimy, size_t dimz, size_t value)
		: regular_voxel_storage(ox, oy, oz, d, dimx, dimy, dimz)
		, value_(value)
	{
		if (value != 0) {
			bounds_[0] = vec_n<3, size_t>(0U, 0U, 0U);
			bounds_[1] = extents() - make_vec<size_t>(1U, 1U, 1U);
		}
	}

	uint32_t encode() const {
		return value_ << 8 | CK_CONSTANT;
	}

	abstract_voxel_storage* copy(void* location = nullptr) const {
		return new constant_voxel_storage(ox_, oy_, oz_, d_, dimx_, dimy_, dimz_, value_);
	}

	abstract_voxel_storage* copy_as(voxel_desc_t* fmt, void* location = nullptr) const {
		if (fmt->get_size_in_bits() == 1) {
			return new constant_voxel_storage(ox_, oy_, oz_, d_, dimx_, dimy_, dimz_, value_ > 1 ? 1 : value_);
		} else {
			throw std::runtime_error("Not implemented");
		}
	}

	abstract_voxel_storage* empty_copy() const {
		// @todo is this safe?
		return nullptr;
	}

	abstract_voxel_storage* empty_copy_as(voxel_desc_t* fmt) const {
		// @todo is this safe?
		return nullptr;
	}

	bool is_constant() { return true; }

	size_t value() const {
		return value_;
	}

	void Set(const vec_n<3, size_t>&) {
		throw std::runtime_error("Invalid");
	}

	void Set(const vec_n<3, size_t>&, void*) {
		throw std::runtime_error("Invalid");
	}

	bool Get(const vec_n<3, size_t>&) const {
		return !!value_;
	}

	void Get(const vec_n<3, size_t>& pos, void* loc) const {
		*((typename T::value_type_non_ref*)loc) = value_;
	}

	abstract_voxel_storage* inverted(void* location = nullptr) const {
		throw std::runtime_error("Not implemented");
	}

	abstract_voxel_storage* boolean_union(const abstract_voxel_storage* other_) {
		throw std::runtime_error("Invalid");
	}

	abstract_voxel_storage* boolean_subtraction(const abstract_voxel_storage* other_) {
		throw std::runtime_error("Invalid");
	}

	abstract_voxel_storage* boolean_intersection(const abstract_voxel_storage* other_) {
		throw std::runtime_error("Invalid");
	}

	void boolean_union_inplace(const abstract_voxel_storage* other_) {
		throw std::runtime_error("Invalid");
	}

	void boolean_subtraction_inplace(const abstract_voxel_storage* other_) {
		throw std::runtime_error("Invalid");
	}

	void boolean_intersection_inplace(const abstract_voxel_storage* other_) {
		throw std::runtime_error("Invalid");
	}

	bool is_explicit() { return false; }

	continuous_voxel_storage<T>* make_explicit(void* location = nullptr) const {
		continuous_voxel_storage<T>* c = new continuous_voxel_storage<T>(ox_, oy_, oz_, d_, dimx_, dimy_, dimz_, location);
		if (value_ != 0) {
			BEGIN_LOOP(size_t(0), dimx_, 0U, dimy_, 0U, dimz_)
				c->Set(ijk);
			END_LOOP;
		}

		return c;
	}

	void write(file_part p, std::ostream& os) {
		if (p == file_part_primitives) {
			os << "CONST(" << value_ << ")";
		}
	}

	long long unsigned int count() const {
		if (value_ == 0) {
			return 0;
		} else {
			return dimx_ * dimy_ * dimz_;
		}
	}
};

template <typename T, size_t N>
class multi_dim_array {
private:
	T * ts_;
	std::array<size_t, N> dims_;
	bool mapped_;

	template<int i = 0, long long M = N, typename... Tp>
	inline typename std::enable_if < i == M || i < 0, size_t>::type reduce_multiply_() const {
		return 1;
	}

	template<int i = 0, long long M = N, typename... Tp>
	inline typename std::enable_if< i < M && i >= 0, size_t>::type reduce_multiply_() const {
		return dims_.at(i) * reduce_multiply_<i + 1, M>();
	}

	template<int i = 0, long long M = N, typename... Tp>
	inline typename std::enable_if<i == M, size_t>::type calculate_index_(const std::tuple<Tp...>& idx) const {
		return 0;
	}

	template<int i = 0, long long M = N, typename... Tp>
	inline typename std::enable_if< i < M, size_t>::type calculate_index_(const std::tuple<Tp...>& idx) const {
#ifndef NDEBUG
		if (std::get<i>(idx) >= dims_.at(i)) {
			throw std::runtime_error("Index out of bounds");
		}
#endif
		return reduce_multiply_<0, i>() * std::get<i>(idx) + calculate_index_<i + 1, M>(idx);
	}

public:
	typedef T element_type;

	multi_dim_array()
		: ts_(nullptr), mapped_(false) {}

	~multi_dim_array() {
		if (!mapped_) {
			delete ts_;
		}
	}

	template <typename... DimsT>
	void resize(void* location, DimsT... dims) {
		static_assert(N == sizeof...(dims), "Number of dimension does not match");
		dims_ = { dims... };
		size_t total = reduce_multiply_();

		if (!mapped_) {
			delete ts_;
		}

		if (location != nullptr) {
			ts_ = new (location) T[total];
			mapped_ = true;
		} else {
			ts_ = new T[total]{ 0 };
		}
	}

	template <typename... DimsT>
	multi_dim_array(void* location, DimsT... dims) {
		resize(location, dims...);
	}

	template <typename... PosT>
	T& get(PosT... i) {
		static_assert(N == sizeof...(i), "Number of dimension does not match");
		auto tup = std::make_tuple(i...);
		return ts_[calculate_index_(tup)];
	}

	template <typename... PosT>
	const T& get(PosT... i) const {
		static_assert(N == sizeof...(i), "Number of dimension does not match");
		auto tup = std::make_tuple(i...);
		return ts_[calculate_index_(tup)];
	}

	const std::array<size_t, N>& dimensions() const {
		return dims_;
	}

	T*& begin() {
		return &ts_[0];
	}

	T* cbegin() const {
		return &ts_[0];
	}

	T*& end() {
		return &ts_[reduce_multiply_() - 1];
	}

	T* cend() const {
		return &ts_[reduce_multiply_() - 1];
	}

	size_t count() const {
		return reduce_multiply_();
	}

	size_t size_bytes() const {
		return sizeof(element_type) * reduce_multiply_();
	}
};

#include <boost/iostreams/device/mapped_file.hpp>

namespace io = boost::iostreams;

class abstract_chunked_voxel_storage : public regular_voxel_storage {
protected:
	// hack for multithreading
	bool bounds_locked_;

	size_t chunk_size_, nchunksx_, nchunksy_, nchunksz_, total_nchunks_;
	vec_n<3, long> grid_offset_;

	inline void validate_chunk_index(const vec_n<3, size_t>& c) const {
#ifndef NDEBUG
		if ((c >= vec_n<3, size_t>(nchunksx_, nchunksy_, nchunksz_)).any()) {
			throw std::runtime_error("Chunk index out of range");
		}
#endif
	}

	/////
	// Some helper stuff for boolean operations

	virtual abstract_chunked_voxel_storage* new_with_size(vec_n<3, long> idx, vec_n<3, size_t> numchunks) = 0;

	enum boolean_mode { OP_UNION, OP_SUBTRACTION, OP_INTERSECTION };
	enum chunk_status { CK_EMPTY, CK_MIXED_IMPL, CK_MIXED_EXPL, CK_FULL };

	virtual void* next_slot() {
		return nullptr;
	}

	static abstract_voxel_storage* get_chunk(const abstract_chunked_voxel_storage* this_, const vec_n<3, long>& idx) {
		auto idx_local = idx - this_->grid_offset_;
		return this_->get_chunk(idx_local.as<size_t>());
	}

	static void set_chunk(abstract_chunked_voxel_storage* this_, const vec_n<3, long>& idx, abstract_voxel_storage* c) {
		auto idx_local = idx - this_->grid_offset_;
		// @todo free
		this_->set_chunk(idx_local.as<size_t>(), c);
	}

	static chunk_status get_chunk_status_local(const abstract_chunked_voxel_storage* this_, const vec_n<3, size_t>& idx) {
		auto c = this_->get_chunk(idx);
		auto cs = this_->chunk_size_;
		if (c == nullptr || c->count() == 0) {
			return CK_EMPTY;
		} else if (c->count() == cs * cs * cs) {
			return CK_FULL;
		} else if (c->is_explicit()) {
			return CK_MIXED_EXPL;
		} else {
			return CK_MIXED_IMPL;
		}
	}

	static chunk_status get_chunk_status(const abstract_chunked_voxel_storage* this_, const vec_n<3, long>& idx) {
		auto idx_local = idx - this_->grid_offset_;
		if ((idx_local < make_vec<long>(0, 0, 0)).any() || (idx_local.as<size_t>() >= this_->num_chunks()).any()) {
			return CK_EMPTY;
		}
		return get_chunk_status_local(this_,idx_local.as<size_t>());
	}

	abstract_voxel_storage* make_explicit(void* location = nullptr) const {
		throw std::runtime_error("Not implemented");
	}

	abstract_voxel_storage* inverted(void* location = nullptr) const {
		auto zero = make_vec<size_t>(0U, 0U, 0U);
		abstract_chunked_voxel_storage* n = (abstract_chunked_voxel_storage*) empty_copy();
		BEGIN_LOOP2(zero, num_chunks())
			chunk_status Sa = get_chunk_status_local(this, ijk);
			abstract_voxel_storage* to_delete = nullptr;
			abstract_voxel_storage* inv;
			switch (Sa) {
			case CK_FULL:
				// Inversion of full chunk is empty so can remain unset
				break;
			case CK_EMPTY:
				n->create_constant(ijk, 1);
				break;
			case CK_MIXED_IMPL:
				to_delete = get_chunk(ijk)->make_explicit();
			case CK_MIXED_EXPL:
				inv = (to_delete ? to_delete : get_chunk(ijk))->inverted(n->next_slot());
				n->set_chunk(ijk, inv);
			}
			delete to_delete;
		END_LOOP

		return n;
	}

	abstract_voxel_storage* boolean_operation(const abstract_chunked_voxel_storage* other, boolean_mode mode, bool inplace=false) {
		abstract_chunked_voxel_storage* n;

		auto left = grid_offset_;
		auto right = left + num_chunks().as<long>();

		if (std::fabs(voxel_size() - other->voxel_size()) > 1.e-7) {
			throw std::runtime_error("Cowardly refusing to perform different boolean operation on differently spaced grids");
		}

		if (inplace) {
			if ((grid_offset_ != other->grid_offset_).any() || (num_chunks() != other->num_chunks()).any()) {
				throw std::runtime_error("Dimensions need to match for in-place boolean operations");
			}

			n = this;
		} else {			
			left.inplace_minimum(other->grid_offset_);
			right.inplace_maximum(other->grid_offset_ + other->num_chunks().as<long>());
			n = new_with_size(left, (right - left).as<size_t>());
		}

		BEGIN_LOOP2(left, right)
			chunk_status Sa = get_chunk_status(this, ijk);
			chunk_status Sb = get_chunk_status(other, ijk);

			if (Sa == CK_EMPTY && Sb == CK_EMPTY) {
				continue;
			}

			auto a = Sa == CK_EMPTY ? nullptr : get_chunk(this, ijk);
			auto b = Sb == CK_EMPTY ? nullptr : get_chunk(other, ijk);

			// static const char* const css[] =  { "CK_EMPTY", "CK_MIXED_IMPL", "CK_MIXED_EXPL", "CK_FULL" };
			// std::cout << "boolean operatoin of chunk " << ijk.format() << " " << css[Sa] << " " << css[Sb] << std::endl;  

			if (mode == OP_UNION) {
				if (Sb == CK_FULL && get_chunk(n, ijk) == nullptr && b->value_bits() == 1) {
					n->create_constant((ijk - left).as<size_t>(), 1U);
					continue;
				} else if (Sb == CK_EMPTY) {
					if (!inplace) {
						set_chunk(n, ijk, a->copy(n->next_slot()));
					}
					continue;
				} else if ((Sa == CK_EMPTY || Sb == CK_FULL) && n->value_bits() == b->value_bits()) {
					// When copying from b we need to check whether datatypes match
					set_chunk(n, ijk, b->copy(n->next_slot()));
					continue;
				}
			} else if (mode == OP_SUBTRACTION) {
				if (Sa == CK_EMPTY) {
					continue;
				} else if (Sb == CK_FULL) {
					set_chunk(n, ijk, nullptr);
					continue;
				} else if (Sb == CK_EMPTY) {
					if (!inplace) {
						set_chunk(n, ijk, a->copy(n->next_slot()));
					}
					continue;
				}
			} else if (mode == OP_INTERSECTION) {
				if (Sa == CK_EMPTY) {
					continue;
				} else if (Sb == CK_EMPTY) {
					set_chunk(n, ijk, nullptr);
					continue;
				} else if (Sb == CK_FULL) {
					if (!inplace) {
						set_chunk(n, ijk, a->copy(n->next_slot()));
					}
					continue;
				} else if (Sa == CK_FULL && n->value_bits() == b->value_bits()) {
					set_chunk(n, ijk, b->copy(n->next_slot()));
					continue;
				}
			}

			// @todo superimpose planar primitives

			if (inplace && get_chunk(this, ijk) != nullptr) {
				release(get_chunk(this, ijk));
			}

			abstract_voxel_storage* a_explicit;
			if (a == nullptr) {
				a_explicit = a = n->get_or_create_chunk(ijk.as<size_t>());
			} else {
				a_explicit = a->is_explicit() ? (inplace ? a : a->copy(n->next_slot())) : a->make_explicit(n->next_slot());
			}
			// b is temporary, so not assigned next_slot
			abstract_voxel_storage* b_temp = b->is_explicit() ? nullptr : b->make_explicit();
			abstract_voxel_storage* b_forced_explicit = b_temp ? b_temp : b;
			if (mode == OP_UNION) {
				a_explicit->boolean_union(b_forced_explicit);
			} else if (mode == OP_SUBTRACTION) {
				a_explicit->boolean_subtraction(b_forced_explicit);
			} else if (mode == OP_INTERSECTION) {
				a_explicit->boolean_intersection(b_forced_explicit);
			}
			delete b_temp;

			if (inplace && a_explicit != a) {
				delete a;
			}
			if (!inplace || a_explicit != a) {
				set_chunk(n, ijk, a_explicit);
			}			
			
		END_LOOP;

		return n;
	}

	/////

public:
	size_t chunk_size() const { return chunk_size_; }

	vec_n<3> num_chunks() const { return make_vec(nchunksx_, nchunksy_, nchunksz_); }

	vec_n<3, long> grid_offset() const {
		return grid_offset_;
	}

	abstract_chunked_voxel_storage(const vec_n<3, long>& grid_offset, double d, size_t chunk_size, const vec_n<3, size_t>& num_chunks)
		: regular_voxel_storage(
			grid_offset.get<0>() * (long) chunk_size * d,
			grid_offset.get<1>() * (long) chunk_size * d,
			grid_offset.get<2>() * (long) chunk_size * d,
			d,
			num_chunks.get<0>() * chunk_size,
			num_chunks.get<1>() * chunk_size,
			num_chunks.get<2>() * chunk_size)
		, bounds_locked_(false)
		, chunk_size_(chunk_size)
		, nchunksx_(num_chunks.get<0>())
		, nchunksy_(num_chunks.get<1>())
		, nchunksz_(num_chunks.get<2>())
		, total_nchunks_(nchunksx_ * nchunksy_ * nchunksz_)
		, grid_offset_(grid_offset)
	{}

	abstract_chunked_voxel_storage(double ox, double oy, double oz, double d, size_t dimx, size_t dimy, size_t dimz, size_t chunk_size)
		// to be overwritten in function body
		: regular_voxel_storage(ox, oy, oz, d, dimx, dimy, dimz)
		, bounds_locked_(false)
		, chunk_size_(chunk_size)
		, nchunksx_(0)
		, nchunksy_(0)
		, nchunksz_(0)
		, total_nchunks_(0)
	{
		// NB: Floor is necessary here as conversion to long does trunc()
		grid_offset_ = (origin_ / (d * chunk_size_)).floor().as<long>();

		// align origin to global chunk grid
		auto aligned_origin = grid_offset_.as<double>() * (d_ * chunk_size_);
		auto origin_difference_in_voxels = ((origin() - aligned_origin) / d).ceil().as<size_t>();

		origin_.get<0>() = ox_ = aligned_origin.get<0>();
		origin_.get<1>() = oy_ = aligned_origin.get<1>();
		origin_.get<2>() = oz_ = aligned_origin.get<2>();

		// increase dimensions to account for shifted origin
		dimx_ += origin_difference_in_voxels.get<0>();
		dimy_ += origin_difference_in_voxels.get<1>();
		dimz_ += origin_difference_in_voxels.get<2>();

		// align voxel extents to chunk grid
		nchunksx_ = integer_ceil_div(dimx_, chunk_size_);
		nchunksy_ = integer_ceil_div(dimy_, chunk_size_);
		nchunksz_ = integer_ceil_div(dimz_, chunk_size_);

		dimx_ = nchunksx_ * chunk_size_;
		dimy_ = nchunksy_ * chunk_size_;
		dimz_ = nchunksz_ * chunk_size_;

		total_nchunks_ = nchunksx_ * nchunksy_ * nchunksz_;
	}

	void lock_bounds() {
		bounds();
		bounds_locked_ = true;
	}

	void unlock_bounds() {
		bounds_locked_ = false;
	}

	virtual abstract_voxel_storage* get_chunk(const vec_n<3, size_t>& pos) const = 0;
	virtual abstract_voxel_storage* get_or_create_chunk(const vec_n<3, size_t>& pos) = 0;
	virtual void set_chunk(const vec_n<3, size_t>& pos, abstract_voxel_storage*) = 0;

	void Set(const vec_n<3, size_t>& xyz) {
		vec_n<3, size_t> cxyz = xyz / chunk_size_;
		auto c = get_or_create_chunk(cxyz);
		if (!c->is_explicit()) {
			auto c2 = c->make_explicit(next_slot());
			delete c;
			c = c2;
			set_chunk(cxyz, c2);
		}
		c->Set(xyz - cxyz * chunk_size_);
	}

	void Set(const vec_n<3, size_t>& xyz, void* v) {
		vec_n<3, size_t> cxyz = xyz / chunk_size_;
		auto c = get_or_create_chunk(cxyz);
		if (!c->is_explicit()) {
			auto c2 = c->make_explicit(next_slot());
			delete c;
			c = c2;
			set_chunk(cxyz, c2);
		}
		c->Set(xyz - cxyz * chunk_size_, v);
	}

	abstract_voxel_storage* boolean_union(const abstract_voxel_storage* other_) {
		const abstract_chunked_voxel_storage* other = (const abstract_chunked_voxel_storage*) other_;
		return boolean_operation(other, OP_UNION);
	}

	abstract_voxel_storage* boolean_subtraction(const abstract_voxel_storage* other_) {
		const abstract_chunked_voxel_storage* other = (const abstract_chunked_voxel_storage*)other_;
		return boolean_operation(other, OP_SUBTRACTION);
	}

	abstract_voxel_storage* boolean_intersection(const abstract_voxel_storage* other_) {
		const abstract_chunked_voxel_storage* other = (const abstract_chunked_voxel_storage*)other_;
		return boolean_operation(other, OP_INTERSECTION);
	}

	void boolean_union_inplace(const abstract_voxel_storage* other_) {
		const abstract_chunked_voxel_storage* other = (const abstract_chunked_voxel_storage*)other_;
		boolean_operation(other, OP_UNION, true);
	}

	void boolean_subtraction_inplace(const abstract_voxel_storage* other_) {
		const abstract_chunked_voxel_storage* other = (const abstract_chunked_voxel_storage*)other_;
		boolean_operation(other, OP_SUBTRACTION, true);
	}

	void boolean_intersection_inplace(const abstract_voxel_storage* other_) {
		const abstract_chunked_voxel_storage* other = (const abstract_chunked_voxel_storage*)other_;
		boolean_operation(other, OP_INTERSECTION, true);
	}

	void create_constant(vec_n<3, size_t> c, size_t value) {
		if (get_chunk(c) != nullptr) {
			throw std::runtime_error("Invalid");
		}
		if (value == 1) {
			vec_n<3, double> O = vec_n<3, double>(ox_, oy_, oz_) + (c * chunk_size_).as<double>() * d_;
			set_chunk(c, new constant_voxel_storage<bit_t>(
				O.get(0), O.get(1), O.get(2),
				d_, chunk_size_, chunk_size_, chunk_size_, value));
		}
	}

	bool create_plane_primitive(vec_n<3, size_t> c, size_t axis, size_t offset) {
		if (get_chunk(c) != nullptr) {
			if (get_chunk(c)->is_explicit()) {
				throw std::runtime_error("Invalid");
			}

			planar_voxel_storage<bit_t>* p = (planar_voxel_storage<bit_t>*) get_chunk(c);
			if (p->axis() != axis) {
				return false;
			} else {
				p->add(offset);
				return true;
			}
		} else {
			vec_n<3, double> O = vec_n<3, double>(ox_, oy_, oz_) + (c * chunk_size_).as<double>() * d_;
			set_chunk(c, new planar_voxel_storage<bit_t>(
				O.get(0), O.get(1), O.get(2),
				d_, chunk_size_, chunk_size_, chunk_size_, axis, offset));
			return true;
		}
	}

	virtual const std::array< vec_n<3, size_t>, 2 >& bounds() const {
		if (bounds_locked_) {
			return bounds_;
		}

		const size_t mi = std::numeric_limits<size_t>::min();
		const size_t ma = std::numeric_limits<size_t>::max();

		decltype(bounds_) temp_bounds;

		temp_bounds[0] = vec_n<3, size_t>(ma, ma, ma);
		temp_bounds[1] = vec_n<3, size_t>(mi, mi, mi);

		BEGIN_LOOP(size_t(0), nchunksx_, 0U, nchunksy_, 0U, nchunksz_)
			if (get_chunk(ijk) != nullptr && get_chunk(ijk)->count() > 0) {
				vec_n<3, size_t> offset = ijk * chunk_size_;

				std::array< vec_n<3, size_t>, 2 > other_bounds = get_chunk(ijk)->bounds();
				other_bounds[0] += offset;
				other_bounds[1] += offset;

				temp_bounds[0].inplace_minimum(other_bounds[0]);
				temp_bounds[1].inplace_maximum(other_bounds[1]);
			}
		END_LOOP;

		bounds_ = temp_bounds;

		return bounds_;
	}

	void write(file_part p, std::ostream& os) {
		if (p == file_part_index) {
			BEGIN_LOOP(size_t(0), nchunksx_, 0U, nchunksy_, 0U, nchunksz_)
				char w = 0;
				auto c = get_chunk(ijk);
				w = c != nullptr;
				if (c && !c->is_explicit()) {
					w++;
				}
				os.put(w);
			END_LOOP;
		} else if (p == file_part_contents) {
			BEGIN_LOOP(size_t(0), nchunksx_, 0U, nchunksy_, 0U, nchunksz_)
				auto c = get_chunk(ijk);
				if (c && c->is_explicit()) {
					c->write(p, os);
				}
			END_LOOP;
		} else if (p == file_part_primitives) {
			BEGIN_LOOP(size_t(0), nchunksx_, 0U, nchunksy_, 0U, nchunksz_)
				auto c = get_chunk(ijk);
				if (c && !c->is_explicit()) {
					c->write(p, os);
					os << std::endl;
				}
			END_LOOP;
		} else if (p == file_part_meta) {
			os << "CHUNK2" << std::endl
				<< d_ << std::endl
				<< chunk_size_ << std::endl
				<< origin_.get<0>() << ";" << origin_.get<1>() << ";" << origin_.get<2>() << std::endl
				<< nchunksx_ << ";" << nchunksy_ << ";" << nchunksz_ << std::endl;
		}
	}
};

// @todo why is this not a template class?
class memory_mapped_chunked_voxel_storage : public abstract_chunked_voxel_storage {
private:
	std::string filename_;
	io::mapped_file file_;
	size_t block_size_;
	size_t blocks_requested_;
	multi_dim_array<uint64_t, 3> chunk_index_;
	multi_dim_array<abstract_voxel_storage*, 3> chunk_pointers_;
	std::vector<uint64_t> freed_;
	char* next_slot_;

	virtual void* next_slot() {
		if (!freed_.empty()) {
			auto ptr = file_.data() + freed_.back();
			freed_.pop_back();
			return ptr;
		}

		if (blocks_requested_ >= total_nchunks_) {
			throw std::runtime_error("mmaped space depleted");
		}
		auto n = next_slot_;
		next_slot_ += block_size_;
		blocks_requested_++;
		return n;
	}

	void release(abstract_voxel_storage* c) {
		auto cc = dynamic_cast<continuous_voxel_storage<bit_t>*>(c);
		if (cc) {
			freed_.push_back((char*)cc->data() - file_.data());
			cc->unmap();
		} else {
			throw std::runtime_error("Can only release storage for continuous blocks");
		}
	}

	void initialize_() {
		blocks_requested_ = 0;

		block_size_ = chunk_size_ * chunk_size_ * integer_ceil_div(chunk_size_, 8U / bit_t::size_in_bits);

		if (total_nchunks_ > ((1 << 30) - 1)) {
			throw std::runtime_error("Number of chunks too large");
		}

		// @note: this is just to be able to calcute header size
		chunk_index_.resize(nullptr, nchunksx_, nchunksy_, nchunksz_);

		io::basic_mapped_file_params<std::string> params;
		params.flags = io::mapped_file_base::readwrite;
		params.path = filename_;

		bool file_exists;
		{
			std::ifstream fs(filename_.c_str());
			file_exists = fs.good();
		}
		// if (!file_exists) {
		// @ todo force new files for now		
		params.new_file_size = header_size() + total_nchunks_ * block_size_;
		// }
		
		try {
			file_.open(params);
		} catch (std::exception& e) {
			throw std::runtime_error(e.what());
		}
		
		next_slot_ = file_.data() + header_size();

		chunk_index_.resize(file_.data(), nchunksx_, nchunksy_, nchunksz_);
		chunk_pointers_.resize(nullptr, nchunksx_, nchunksy_, nchunksz_);

		BEGIN_LOOP(size_t(0), nchunksx_, 0U, nchunksy_, 0U, nchunksz_)
			if (chunk_index_.get(ijk.get<0>(), ijk.get<1>(), ijk.get<2>())) {
				uint32_t ty = chunk_index_.get(ijk.get<0>(), ijk.get<1>(), ijk.get<2>()) >> 32;
				uint32_t da = chunk_index_.get(ijk.get<0>(), ijk.get<1>(), ijk.get<2>()) & 0xFFFFFFFF;
				auto O = origin() + ijk.as<double>() * voxel_size();
				if (ty == CK_EXPLICIT) {
					chunk_pointers_.get(ijk.get<0>(), ijk.get<1>(), ijk.get<2>()) = new continuous_voxel_storage<bit_t>(
						O.get<0>(), O.get<1>(), O.get<2>(),
						voxel_size(),
						chunk_size_, chunk_size_, chunk_size_, file_.data() + da);

					if (file_.data() + da >= next_slot_) {
						next_slot_ = file_.data() + da + block_size_;
					}
				} else {
					throw std::runtime_error("@todo");
				}
			}
		END_LOOP;
	}

	abstract_chunked_voxel_storage* new_with_size(vec_n<3, long> idx, vec_n<3, size_t> numchunks) {
		return new memory_mapped_chunked_voxel_storage(idx, d_, chunk_size_, numchunks, factory::mmap_filename());
	}

public:
	using abstract_chunked_voxel_storage::get_chunk;

	memory_mapped_chunked_voxel_storage(double ox, double oy, double oz, double d, size_t dimx, size_t dimy, size_t dimz, size_t chunk_size, const std::string& filename)
		: abstract_chunked_voxel_storage(ox, oy, oz, d, dimx, dimy, dimz, chunk_size)
		, filename_(filename)
	{
		initialize_();
	}

	memory_mapped_chunked_voxel_storage(const vec_n<3, long>& grid_offset, double d, size_t chunk_size, const vec_n<3, size_t>& num_chunks, const std::string& filename)
		: abstract_chunked_voxel_storage(grid_offset, d, chunk_size, num_chunks)
		, filename_(filename)
	{
		initialize_();
	}
	
	size_t header_size() const {
		return chunk_index_.size_bytes();
	}

	long long unsigned int count() const {
		long long unsigned int n = 0;
		for (auto it = chunk_pointers_.cbegin(); it != chunk_pointers_.cend(); ++it) {
			if ((*it) != nullptr) {
				n += (*it)->count();
			}
		}
		return n;
	}

	inline abstract_voxel_storage* get_chunk(const vec_n<3, size_t>& xyz) const final {
		return chunk_pointers_.get(xyz.get<0>(), xyz.get<1>(), xyz.get<2>());
	}

	bool Get(const vec_n<3, size_t>& xyz) const {
		vec_n<3, size_t> cxyz = xyz / chunk_size_;
		abstract_voxel_storage* c = get_chunk(cxyz);
		if (c == nullptr) {
			return false;
		}
		return c->Get(xyz - cxyz * chunk_size_);
	}

	void Get(const vec_n<3, size_t>& xyz, void* loc) const {
		vec_n<3, size_t> cxyz = xyz / chunk_size_;
		abstract_voxel_storage* c = get_chunk(cxyz);
		if (c == nullptr) {
			throw std::runtime_error("Not implemented, no template arg");
		}
		c->Get(xyz - cxyz * chunk_size_, loc);
	}

	abstract_voxel_storage* empty_copy() const {
		auto nc = num_chunks();
		return new memory_mapped_chunked_voxel_storage(grid_offset_, d_, chunk_size_, nc, factory::mmap_filename());
	}

	abstract_voxel_storage* empty_copy_as(voxel_desc_t* fmt) const {
		throw std::runtime_error("not implemented");
	}

	abstract_voxel_storage* copy(void* location = nullptr) const {
		auto nc = num_chunks();
		memory_mapped_chunked_voxel_storage* c = new memory_mapped_chunked_voxel_storage(grid_offset_, d_, chunk_size_, nc, factory::mmap_filename());

		BEGIN_LOOP(size_t(0), nchunksx_, 0U, nchunksy_, 0U, nchunksz_)
			if (get_chunk(ijk) != nullptr) {
				c->set_chunk(ijk, get_chunk(ijk)->copy(c->next_slot()));
			}
		END_LOOP;

		return c;
	}

	abstract_voxel_storage* copy_as(voxel_desc_t* fmt, void* location = nullptr) const {
		auto nc = num_chunks();
		memory_mapped_chunked_voxel_storage* c = new memory_mapped_chunked_voxel_storage(grid_offset_, d_, chunk_size_, nc, factory::mmap_filename());

		BEGIN_LOOP(size_t(0), nchunksx_, 0U, nchunksy_, 0U, nchunksz_)
			if (get_chunk(ijk) != nullptr) {
				c->set_chunk(ijk, get_chunk(ijk)->copy(c->next_slot()));
			}
		END_LOOP;

		return c;
	}

	abstract_voxel_storage* get_or_create_chunk(const vec_n<3, size_t>& ijk) {
		const size_t& i = ijk.get<0>();
		const size_t& j = ijk.get<1>();
		const size_t& k = ijk.get<2>();

		if (chunk_pointers_.get(i, j, k) == nullptr) {
			auto O = origin() + vec_n<3, size_t>(i, j, k).as<double>() * voxel_size();
			continuous_voxel_storage<bit_t>* c;
			chunk_pointers_.get(i, j, k) = c = new continuous_voxel_storage<bit_t>(
				O.get<0>(), O.get<1>(), O.get<2>(),
				voxel_size(),
				chunk_size_, chunk_size_, chunk_size_, next_slot());
			chunk_index_.get(i, j, k) = (uint32_t) ((char*) c->data() - file_.data());
		}

		return chunk_pointers_.get(i, j, k);
	}

	void set_chunk(const vec_n<3, size_t>& ijk, abstract_voxel_storage* s) {
		const size_t& i = ijk.get<0>();
		const size_t& j = ijk.get<1>();
		const size_t& k = ijk.get<2>();

		// @todo can s be null?

		// if (!freed_.empty()) { ...
		
		chunk_pointers_.get(i, j, k) = s;
		if (s->is_explicit()) {
			auto c = (continuous_voxel_storage<bit_t>*)s;
			if (!c->is_mapped()) {
				throw std::runtime_error("Storage not mapped");
			}
			if (((char*)c->data() < file_.data()) || ((char*)c->data() >= (file_.data() + total_nchunks_ * block_size_))) {
				throw std::runtime_error("Out of mapped storage range");
			}
			chunk_index_.get(i, j, k) = (uint32_t)((char*)c->data() - file_.data());
		} else {
			chunk_index_.get(i, j, k) = ((uint64_t) s->encode()) << 32;
		}
	}
};

template <typename T>
class chunked_voxel_storage : public abstract_chunked_voxel_storage {
private:
	abstract_voxel_storage** chunks_;
	
	abstract_chunked_voxel_storage* new_with_size(vec_n<3, long> idx, vec_n<3, size_t> numchunks) {
		return new chunked_voxel_storage(idx, d_, chunk_size_, numchunks);
	}

public:
	virtual int value_bits() const { return T::size_in_bits; }

	chunked_voxel_storage(double ox, double oy, double oz, double d, size_t dimx, size_t dimy, size_t dimz, size_t chunk_size)
		: abstract_chunked_voxel_storage(ox, oy, oz, d, dimx, dimy, dimz, chunk_size)
		, chunks_(new abstract_voxel_storage*[total_nchunks_] {0})
	{}

	chunked_voxel_storage(const vec_n<3, long>& grid_offset, double d, size_t chunk_size, const vec_n<3, size_t>& num_chunks)
		: abstract_chunked_voxel_storage(grid_offset, d, chunk_size, num_chunks)
		, chunks_(new abstract_voxel_storage*[total_nchunks_] {0}) 
	{}

	~chunked_voxel_storage() {
		for (size_t i = 0; i < total_nchunks_; ++i) {
			delete chunks_[i];
		}
		delete[] chunks_;
	}

	abstract_voxel_storage* empty_copy() const {
		auto nc = num_chunks();
		return new chunked_voxel_storage(grid_offset_, d_, chunk_size_, nc);
	}

	abstract_voxel_storage* empty_copy_as(voxel_desc_t* fmt) const {
		auto nc = num_chunks();
		// @todo some sort of factory or visitor is needed here:
		if (fmt->get_size_in_bits() == 1) {
			return new chunked_voxel_storage<bit_t>(grid_offset_, d_, chunk_size_, nc);
		} else if (fmt->get_size_in_bits() == 8) {
			return new chunked_voxel_storage<voxel_uint8_t>(grid_offset_, d_, chunk_size_, nc);
		} else if (fmt->get_size_in_bits() == 32) {
			return new chunked_voxel_storage<voxel_uint32_t>(grid_offset_, d_, chunk_size_, nc);
		} else {
			throw std::runtime_error("Not implemented");
		}
	}

	abstract_voxel_storage* copy(void* location = nullptr) const {
		auto nc = num_chunks();
		chunked_voxel_storage* c = new chunked_voxel_storage(grid_offset_, d_, chunk_size_, nc);

		BEGIN_LOOP(size_t(0), nchunksx_, 0U, nchunksy_, 0U, nchunksz_)
			if (get_chunk(ijk) != nullptr) {
				c->set_chunk(ijk, get_chunk(ijk)->copy());
			}
		END_LOOP;

		return c;
	}

	abstract_voxel_storage* copy_as(voxel_desc_t* fmt, void* location = nullptr) const {
		if (fmt->get_size_in_bits() == 1) {
			
			auto nc = num_chunks();
			chunked_voxel_storage<bit_t>* c = new chunked_voxel_storage<bit_t>(grid_offset_, d_, chunk_size_, nc);

			BEGIN_LOOP(size_t(0), nchunksx_, 0U, nchunksy_, 0U, nchunksz_)
				if (get_chunk(ijk) != nullptr) {
					c->set_chunk(ijk, get_chunk(ijk)->copy_as(fmt));
				}
			END_LOOP;

			return c;

		}
		else {
			throw std::runtime_error("Not implemented");
		}
	}

	abstract_voxel_storage* get_or_create_chunk(const vec_n<3, size_t>& ijk) {
		validate_chunk_index(ijk);
		const size_t& i = ijk.get<0>();
		const size_t& j = ijk.get<1>();
		const size_t& k = ijk.get<2>();

		abstract_voxel_storage*& c = chunks_[i + j * nchunksx_ + k * nchunksx_ * nchunksy_];
		if (c == nullptr) {
			c = new continuous_voxel_storage<T>(
				ox_ + i * chunk_size_ * d_, oy_ + j * chunk_size_ * d_, oz_ + k * chunk_size_ * d_,
				d_, chunk_size_, chunk_size_, chunk_size_);
		}
		return c;
	}

	inline abstract_voxel_storage* get_chunk(const vec_n<3, size_t>& ijk) const final {
		validate_chunk_index(ijk);
		const size_t& i = ijk.get<0>();
		const size_t& j = ijk.get<1>();
		const size_t& k = ijk.get<2>();

		abstract_voxel_storage*& c = chunks_[i + j * nchunksx_ + k * nchunksx_ * nchunksy_];
		return c;
	}

	bool Get(const vec_n<3, size_t>& xyz) const {
		vec_n<3, size_t> cxyz = xyz / chunk_size_;
		abstract_voxel_storage* c = get_chunk(cxyz);
		if (c == nullptr) {
			return false;
		}
		return c->Get(xyz - cxyz * chunk_size_);
	}

	void Get(const vec_n<3, size_t>& xyz, void* loc) const {
		vec_n<3, size_t> cxyz = xyz / chunk_size_;
		abstract_voxel_storage* c = get_chunk(cxyz);
		if (c == nullptr) {
			(*(typename T::value_type_non_ref*) loc) = 0;
		} else {
			c->Get(xyz - cxyz * chunk_size_, loc);
		}
	}
	
	void set_chunk(const vec_n<3, size_t>& ijk, abstract_voxel_storage* s) {
		validate_chunk_index(ijk); 
		const size_t& i = ijk.get<0>();
		const size_t& j = ijk.get<1>();
		const size_t& k = ijk.get<2>();

		chunks_[i + j * nchunksx_ + k * nchunksx_ * nchunksy_] = s;
	}

	void chunk_extents(const vec_n<3, size_t> ijk, std::array< vec_n<3, size_t>, 2 >& out) {
		out[0] = ijk * chunk_size_;
		out[1] = out[0] + (chunk_size_ - 1);
	}

	std::vector< vec_n<3, size_t> > empty_chunks() const {
		std::vector< vec_n<3, size_t> > ret;
		BEGIN_LOOP(size_t(0), nchunksx_, 0U, nchunksy_, 0U, nchunksz_)
			if (get_chunk(ijk) == nullptr) {
				ret.push_back(ijk);
			}
		END_LOOP;
		return ret;
	}

	std::vector< vec_n<3, size_t> > empty_chunks_in(const std::array< vec_n<3, size_t>, 2 >& r, size_t ignore_dim) const {
		// voxel index
		std::vector< vec_n<3, size_t> > ret;

		const vec_n<3, size_t>& ijk0 = r[0];
		const vec_n<3, size_t>& ijk1 = r[1];

		// chunk index
		vec_n<3, size_t> c0 = ijk0 / chunk_size_;
		vec_n<3, size_t> c1 = ijk1 / chunk_size_;

		/*
		// tfk: is this still necessary?
		for (size_t i = 0; i < 3; ++i) {
			if (i != ignore_dim && c0.get(i) == c1.get(i)) {
				// tfk: might still be on the exact edges of the single chunk, but unlikely enough to ignore
				return ret;
			}
		}
		*/

		// index in chunk
		vec_n<3, size_t> m0 = ijk0 % chunk_size_;
		vec_n<3, size_t> m1 = ijk1 % chunk_size_;

		// index in chunk on edge -> 1
		vec_n<3, size_t> m0_0 = (m0 != 0U).template as<size_t>();
		vec_n<3, size_t> m1_0 = (m1 != (chunk_size_ - 1U)).template as<size_t>();

		// direction ortho to plane not shifted
		m0_0.get(ignore_dim) = 0;
		m1_0.get(ignore_dim) = 0;

		// shifted chunk index
		// tfk: don't subtract from unsigned int
		// c0 += m0_0;
		// c1 -= m1_0;

		size_t c0_x, c0_y, c0_z, c1_x, c1_y, c1_z, m0_0_x, m0_0_y, m0_0_z, m1_0_x, m1_0_y, m1_0_z;
		c0.tie(c0_x, c0_y, c0_z);
		c1.tie(c1_x, c1_y, c1_z);
		m0_0.tie(m0_0_x, m0_0_y, m0_0_z);
		m1_0.tie(m1_0_x, m1_0_y, m1_0_z);

		auto ijk = vec_n<3, size_t>();
		auto& i = ijk.get<0>();
		auto& j = ijk.get<1>();
		auto& k = ijk.get<2>();

		for (i = c0_x; i <= c1_x; ++i) {
			if (i == c0_x && m0_0_x) continue;
			if (i == c1_x && m1_0_x) continue;
			for (j = c0_y; j <= c1_y; ++j) {
				if (j == c0_y && m0_0_y) continue;
				if (j == c1_y && m1_0_y) continue;
				for (k = c0_z; k <= c1_z; ++k) {
					if (k == c0_z && m0_0_z) continue;
					if (k == c1_z && m1_0_z) continue;

					if (get_chunk(ijk) == nullptr || !get_chunk(ijk)->is_explicit()) {
						ret.push_back(ijk);
					}
				}
			}
		}

		return ret;
	}

	void create_constant(vec_n<3, size_t> c, size_t value) {
		if (get_chunk(c) != nullptr) {
			throw std::runtime_error("Invalid");
		}
		if (value == 1) {
			vec_n<3, double> O = vec_n<3, double>(ox_, oy_, oz_) + (c * chunk_size_).template as<double>() * d_;
			set_chunk(c, new constant_voxel_storage<T>(
				O.get(0), O.get(1), O.get(2),
				d_, chunk_size_, chunk_size_, chunk_size_, value));
		}
	}

	bool create_plane_primitive(vec_n<3, size_t> c, size_t axis, size_t offset) {
		if (get_chunk(c) != nullptr) {
			if (get_chunk(c)->is_explicit()) {
				throw std::runtime_error("Invalid");
			}

			planar_voxel_storage<T>* p = (planar_voxel_storage<T>*) get_chunk(c);
			if (p->axis() != axis) {
				return false;
			} else {
				p->add(offset);
				return true;
			}
		} else {
			vec_n<3, double> O = vec_n<3, double>(ox_, oy_, oz_) + (c * chunk_size_).template as<double>() * d_;
			set_chunk(c, new planar_voxel_storage<T>(
				O.get(0), O.get(1), O.get(2),
				d_, chunk_size_, chunk_size_, chunk_size_, axis, offset));
			return true;
		}
	}

	long long unsigned int count() const {
		long long unsigned int n = 0;
		const size_t nchunks = nchunksx_ * nchunksy_ * nchunksz_;
		for (size_t i = 0; i < nchunks; ++i) {
			if (chunks_[i] != nullptr) {
				n += chunks_[i]->count();
			}
		}
		return n;
	}	
};

class voxel_region : public regular_voxel_storage {
private:
	std::array< vec_n<3, size_t>, 2 > bounds_subset_;
	regular_voxel_storage* base_;

	voxel_region(const vec_n<3, double>& origin, double d, const vec_n<3, size_t> extents, regular_voxel_storage* base, const std::array< vec_n<3, size_t>, 2 >& bounds)
		: regular_voxel_storage(
			origin.get<0>(),
			origin.get<1>(),
			origin.get<2>(),
			d,
			extents.get<0>(),
			extents.get<1>(),
			extents.get<2>())
		, base_(base)
		, bounds_subset_(bounds)
	{}
public:

	regular_voxel_storage * base() const { return base_; }

	virtual const std::array< vec_n<3, size_t>, 2 >& bounds() const {
		return bounds_subset_;
	}

	virtual const std::array< vec_n<3, size_t>, 2 >& original_bounds() const {
		return base_->bounds();
	}

	static voxel_region* make(regular_voxel_storage* base_, size_t i, size_t N) {
		vec_n<3, double> origin = base_->origin();
		
		std::array<vec_n<3, size_t>, 2> bounds = base_->bounds();
		vec_n<3, size_t> extents = bounds[1] - bounds[0];
		
		// extents are [0, n), bounds are [0, n] 
		extents += 1U;
		
		size_t extents_x = extents.get<0>();
		extents_x /= N;
		origin.get<0>() += i * extents_x * base_->voxel_size();
		
		std::array<vec_n<3, size_t>, 2> bounds_subset = bounds;
		bounds_subset[0] = bounds[0];
		bounds_subset[0].get<0>() += i * extents_x;
				
		if (i == N - 1) {
			extents_x += extents.get<0>() - (extents_x * N);
		}
		extents.get<0>() = extents_x;
		bounds_subset[1].get<0>() = bounds_subset[0].get<0>() + extents_x - 1;

		// std::cerr << "bounds_subset " << bounds_subset[0].format() << " - " << bounds_subset[1].format() << std::endl;

		return new voxel_region(origin, base_->voxel_size(), extents, base_, bounds_subset);
	}
	
	virtual void write(file_part, std::ostream&) {
		throw std::runtime_error("not implemented");
	}

	virtual bool Get(const vec_n<3, size_t>& xyz) const {
		return base_->Get(xyz);
	}

	virtual void Get(const vec_n<3, size_t>& xyz, void* loc) const {
		return base_->Get(xyz, loc);
	}

	abstract_voxel_storage* inverted(void* location = nullptr) const {
		throw std::runtime_error("Not implemented");
	}

	virtual void Set(const vec_n<3, size_t>&) {
		throw std::runtime_error("not implemented");
	}

	virtual void Set(const vec_n<3, size_t>&, void*) {
		throw std::runtime_error("not implemented");
	}

	abstract_voxel_storage* boolean_union(const abstract_voxel_storage* other) {
		throw std::runtime_error("not implemented");
	}

	abstract_voxel_storage* boolean_subtraction(const abstract_voxel_storage* other) {
		throw std::runtime_error("not implemented");
	}

	abstract_voxel_storage* boolean_intersection(const abstract_voxel_storage* other) {
		throw std::runtime_error("not implemented");
	}

	void boolean_union_inplace(const abstract_voxel_storage* other_) {
		throw std::runtime_error("not implemented");
	}

	void boolean_subtraction_inplace(const abstract_voxel_storage* other_) {
		throw std::runtime_error("not implemented");
	}

	void boolean_intersection_inplace(const abstract_voxel_storage* other_) {
		throw std::runtime_error("not implemented");
	}

	virtual abstract_voxel_storage* make_explicit(void* location = nullptr) const {
		throw std::runtime_error("not implemented");
	}

	// @todo are these correctly implemented?
	virtual abstract_voxel_storage* empty_copy() const { return base_->empty_copy(); }
	virtual abstract_voxel_storage* copy(void* location = nullptr) const { return base_->copy(); }

	virtual abstract_voxel_storage* empty_copy_as(voxel_desc_t* fmt) const { throw std::runtime_error("Not implemented"); }
	virtual abstract_voxel_storage* copy_as(voxel_desc_t* fmt, void* location = nullptr) const { throw std::runtime_error("Not implemented"); }

	virtual long long unsigned int count() const {
		unsigned long long n = 0;
		size_t i0, j0, k0, i1, j1, k1;
		bounds()[0].tie(i0, j0, k0);
		bounds()[1].tie(i1, j1, k1);

		BEGIN_LOOP_I(i0, i1, j0, j1, k0, k1) 
			if (Get(ijk)) {
				n += 1;
			}
		END_LOOP;

		return n;
	}
};

regular_voxel_storage* storage_for(std::array< vec_n<3, double>, 2 >& bounds, size_t max_extents = 1024U, size_t padding = 0U, size_t chunk_size = 64U);

namespace {
	bool equal_pointed_to(int w, void* a, void* b) {
		uint8_t* A = (uint8_t*)a;
		uint8_t* B = (uint8_t*)b;
		for (int i = 0; i < w; ++i) {
			if (*A != *B) {
				return false;
			}
			++A, ++B;
		}
		return true;
	}

	bool is_zero(int w, void* a) {
		uint8_t* A = (uint8_t*)a;
		for (int i = 0; i < w; ++i) {
			if (*A != 0) {
				return false;
			}
			++A;
		}
		return true;
	}

	long long to_number(int w, void* a) {
		if (w == 4) {
			// @todo std::is_integral, std::is_signed, ...
			return *((uint32_t*)a);
		} else {
			throw std::runtime_error("Not implemented");
		}
	}
}

namespace std {
	template <>
	struct iterator_traits<set_voxel_iterator> {
		typedef forward_iterator_tag iterator_category;
		typedef std::ptrdiff_t difference_type;
	};
}

#endif
