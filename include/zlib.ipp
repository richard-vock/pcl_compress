namespace pcl_compress {

template <typename T>
void zlib_compress_object(T&& v, std::ostream& output_buffer) {
    std::stringstream data_stream;
    {
        cereal::BinaryOutputArchive ar_out(data_stream);
        ar_out(v);
    }
    data_stream.seekg(0);
    zlib_compress_stream(data_stream, output_buffer);
}

template <typename T>
T zlib_decompress_object(std::istream& input_buffer) {
    std::stringstream data_stream;
    zlib_decompress_stream_heap(input_buffer, data_stream);
    data_stream.seekg(0);
    T v;
    {
        cereal::BinaryInputArchive ar_in(data_stream);
        ar_in(v);
    }
    return v;
}

} // pcl_compress
