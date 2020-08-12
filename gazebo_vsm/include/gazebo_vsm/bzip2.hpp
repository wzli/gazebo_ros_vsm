#pragma once
#include <sstream>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/bzip2.hpp>

namespace bzip2 {

inline std::string compress(const std::string& data) {
    namespace bio = boost::iostreams;

    std::stringstream compressed;
    std::stringstream origin(data);

    bio::filtering_streambuf<bio::input> out;
    out.push(bio::bzip2_compressor(bio::bzip2_params()));
    out.push(origin);
    bio::copy(out, compressed);

    return compressed.str();
}

inline std::string decompress(const std::string& data) {
    namespace bio = boost::iostreams;

    std::stringstream compressed(data);
    std::stringstream decompressed;

    bio::filtering_streambuf<bio::input> out;
    out.push(bio::bzip2_decompressor());
    out.push(compressed);
    bio::copy(out, decompressed);

    return decompressed.str();
}

}  // namespace bzip2
