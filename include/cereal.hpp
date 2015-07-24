#ifndef PCL_COMPRESS_CEREAL_HPP_
#define PCL_COMPRESS_CEREAL_HPP_

#ifdef USE_CEREAL

// general
#include <fstream>
#include <cereal/cereal.hpp>
#include <cereal/access.hpp>
#include <cereal/types/base_class.hpp>
#include <cereal/types/polymorphic.hpp>

// container and such
#include <cereal/types/vector.hpp>
#include <cereal/types/utility.hpp>
#include <cereal/types/tuple.hpp>
#include <cereal/types/memory.hpp>

// archives
#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>
#include <cereal/archives/json.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>


namespace cereal {


template <class Archive, class _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
void save(Archive& ar, const ::Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& m) {
	int rows = m.rows(), cols = m.cols();
	ar(CEREAL_NVP(rows));
	ar(CEREAL_NVP(cols));
	const _Scalar* data = m.data();
	for (uint32_t i=0; i<m.size(); ++i) {
		ar(data[i]);
	}
}

template <class Archive, class _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
void load(Archive& ar, ::Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& m) {
	int rows, cols;
	ar(CEREAL_NVP(rows));
	ar(CEREAL_NVP(cols));
	if (rows * cols != m.size()) m.resize(rows, cols);
	_Scalar value;
	_Scalar* data = m.data();
	for (uint32_t i=0; i<m.size(); ++i) {
		ar(value);
		data[i] = value;
	}
}

template <class Archive, class _Scalar, int _AmbientDim>
inline void serialize(Archive& ar, ::Eigen::AlignedBox<_Scalar, _AmbientDim>& bb) {
	ar & make_nvp("min", bb.min());
	ar & make_nvp("max", bb.max());
}

template <class Archive, class _Scalar, int _AmbientDim, int _Options>
inline void serialize(Archive& ar, ::Eigen::Hyperplane<_Scalar, _AmbientDim, _Options>& plane) {
	ar & make_nvp("coeffs", plane.coeffs());
}

template <class Archive, class _Scalar, int _Options>
inline void serialize(Archive& ar, ::Eigen::Quaternion<_Scalar, _Options>& quat) {
	ar(make_nvp("w", quat.w()), make_nvp("x", quat.x()), make_nvp("y", quat.y()), make_nvp("z", quat.z()));
}


} // cereal

namespace pcl_compress {

typedef enum {BINARY, XML, JSON} archive_t;

template <class Archive>
inline void serialize_values(Archive&) {
}

template <class Archive, typename T, typename... Ts>
inline void serialize_values(Archive& ar, T&& value, Ts&&... values) {
	ar(std::forward<T>(value));
	serialize_values(ar, values...);
}


template <class Archive, typename... Ts>
inline void serialize(const std::string& path, Ts&&... values) {
	std::ofstream out(path.c_str());
	if (!out.good()) {
		throw std::runtime_error("serialize(): Unable to open file \"" + path + "\" for writing.");
	}

	{
		Archive ar(out);

		serialize_values(ar, std::forward<Ts>(values)...);
	}

	out << "\n";

	out.close();
}

template <class Archive, typename... Ts>
inline void deserialize(const std::string& path, Ts&&... values) {
	std::ifstream in(path.c_str());
	if (!in.good()) {
		throw std::runtime_error("deserialize(): Unable to open file \"" + path + "\" for reading.");
	}

	{
		Archive ar(in);

		serialize_values(ar, std::forward<Ts>(values)...);
	}

	in.close();
}

template <typename... Ts>
inline void serialize(archive_t type, const std::string& path, Ts&&... values) {
	switch (type) {
		case BINARY: serialize<cereal::BinaryOutputArchive>(path, std::forward<Ts>(values)...); break;
		case XML:    serialize<cereal::XMLOutputArchive>(path, std::forward<Ts>(values)...); break;
		case JSON:   serialize<cereal::JSONOutputArchive>(path, std::forward<Ts>(values)...); break;
		default: throw std::invalid_argument("serialize(): Unknown archive type. Must be one of {BINARY, XML, JSON}");
	}
}

template <typename... Ts>
inline void deserialize(archive_t type, const std::string& path, Ts&&... values) {
	switch (type) {
		case BINARY: deserialize<cereal::BinaryInputArchive>(path, std::forward<Ts>(values)...); break;
		case XML:    deserialize<cereal::XMLInputArchive>(path, std::forward<Ts>(values)...); break;
		case JSON:   deserialize<cereal::JSONInputArchive>(path, std::forward<Ts>(values)...); break;
		default: throw std::invalid_argument("deserialize(): Unknown archive type. Must be one of {BINARY, XML, JSON}");
	}
}


} // pcl_compress

#endif /* USE_CEREAL */


#ifdef USE_CEREAL
#define CEREAL_ACCESS \
	friend class ::cereal::access;
#else
#define CEREAL_ACCESS
#endif


#endif /* PCL_COMPRESS_CEREAL_HPP_ */
